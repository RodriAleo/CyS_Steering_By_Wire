#!/usr/bin/env python3
"""
Realtime torque-speed operating point viewer for Simulink UDP output.

Expected UDP payload:
  - preferred: five binary doubles [rpm_m, Tm, delta_deg, delta_ref_deg, tiempo],
    little-endian or native endian
  - also accepted: two binary doubles [rpm_m, Tm] for the torque-speed plot only
  - also accepted: text such as "rpm,Tm,delta,delta_ref,time"

Default Simulink setup:
  UDP Send remote address: 127.0.0.1 if Python runs on the same OS as Simulink.
                           Use the WSL IP if Simulink runs on Windows and this
                           script runs inside WSL.
  UDP Send remote port:    5010
  Signal vector:           [rpm_m; Tm; delta_deg; delta_ref_deg; tiempo]

Close the plot window to stop the program.
"""

from __future__ import annotations

import argparse
import socket
import struct
import time
from collections import deque
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import CheckButtons, TextBox


def maybe_iter(value):
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return value
    return [value]


@dataclass
class Sample:
    t_rx: float
    rpm: float
    torque: float
    delta: float | None = None
    delta_ref: float | None = None
    sim_time: float | None = None


def parse_packet(packet: bytes) -> tuple[float, ...] | None:
    """Return common Simulink UDP payload formats."""
    if len(packet) >= 40:
        for fmt in ("<5d", "=5d", ">5d"):
            try:
                values = struct.unpack(fmt, packet[:40])
            except struct.error:
                continue
            if plausible_values(values):
                return values

    if len(packet) >= 16:
        for fmt in ("<2d", "=2d", ">2d"):
            try:
                values = struct.unpack(fmt, packet[:16])
            except struct.error:
                continue
            if plausible_values(values):
                return values

    if len(packet) >= 8:
        for fmt in ("<2f", "=2f", ">2f"):
            try:
                values = struct.unpack(fmt, packet[:8])
            except struct.error:
                continue
            if plausible_values(values):
                return tuple(float(value) for value in values)

    try:
        text = packet.decode("utf-8", errors="ignore").strip()
        for sep in (",", ";", "\t"):
            text = text.replace(sep, " ")
        values = [float(part) for part in text.split()]
    except ValueError:
        return None

    if len(values) >= 5 and plausible_values(values[:5]):
        return tuple(values[:5])

    if len(values) >= 2 and plausible_values(values[:2]):
        return tuple(values[:2])

    return None


def plausible_values(values: tuple[float, ...] | list[float]) -> bool:
    return all(value == value and abs(value) < 1.0e6 for value in values)


def continuous_limit(abs_rpm: float) -> float:
    """Approximate continuous torque envelope for the selected U13060 motor."""
    if abs_rpm <= 400.0:
        return 18.0
    if abs_rpm <= 800.0:
        return 18.0 * (800.0 - abs_rpm) / 400.0
    return 0.0


def peak_limit(abs_rpm: float) -> float:
    """Approximate transient peak torque envelope for the selected U13060 motor."""
    if abs_rpm <= 400.0:
        return 40.0
    if abs_rpm <= 800.0:
        return 40.0 * (800.0 - abs_rpm) / 400.0
    return 0.0


class UdpMotorCurveApp:
    def __init__(
        self,
        host: str,
        port: int,
        stale_timeout_s: float,
        default_window_s: float,
        max_points: int,
    ) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)

        self.stale_timeout_s = stale_timeout_s
        self.window_s = default_window_s
        self.keep_last_seconds = False
        self.samples: deque[Sample] = deque(maxlen=max_points)
        self.last_packet_time: float | None = None
        self.stream_stopped = False
        self.running = True

        self.fig, (self.ax_motor, self.ax_delta) = plt.subplots(
            2,
            1,
            figsize=(10, 7.5),
            gridspec_kw={"height_ratios": [2.0, 1.0]},
        )
        self.fig.canvas.manager.set_window_title("Motor torque-speed operating point")
        self.fig.subplots_adjust(left=0.10, right=0.78, bottom=0.10, top=0.86, hspace=0.34)

        rpm_curve = [0.0, 400.0, 800.0]
        t_cont = [18.0, 18.0, 0.0]
        t_peak = [40.0, 40.0, 0.0]

        self.ax_motor.plot(
            rpm_curve,
            t_cont,
            color="tab:blue",
            linewidth=2.5,
            label="Limite continuo",
        )
        self.ax_motor.plot(
            rpm_curve,
            t_peak,
            color="tab:red",
            linestyle="--",
            linewidth=2.5,
            label="Limite pico transitorio",
        )
        (self.trace_line,) = self.ax_motor.plot(
            [],
            [],
            color="black",
            linewidth=1.2,
            label="Trayectoria instantanea",
        )
        self.point = self.ax_motor.scatter([], [], s=55, color="black", zorder=4)

        self.status_text = self.fig.text(
            0.10,
            0.93,
            "Esperando UDP...",
            va="top",
            ha="left",
            fontsize=10,
        )

        self.ax_motor.set_title("Curva torque-velocidad del motor")
        self.ax_motor.set_xlabel("Velocidad motor |rpm|")
        self.ax_motor.set_ylabel("Torque motor |N.m|")
        self.base_xmax = 850.0
        self.base_ymax = 45.0
        self.ax_motor.set_xlim(0, self.base_xmax)
        self.ax_motor.set_ylim(0, self.base_ymax)
        self.ax_motor.grid(True, alpha=0.35)
        self.ax_motor.legend(loc="upper right")

        (self.delta_line,) = self.ax_delta.plot(
            [],
            [],
            color="tab:blue",
            linewidth=1.5,
            label="delta",
        )
        (self.delta_ref_line,) = self.ax_delta.plot(
            [],
            [],
            color="tab:orange",
            linewidth=1.5,
            label="delta_ref",
        )
        self.ax_delta.set_title("Seguimiento de angulo de rueda")
        self.ax_delta.set_xlabel("Tiempo [s]")
        self.ax_delta.set_ylabel("Angulo [deg]")
        self.ax_delta.set_xlim(0, 1)
        self.ax_delta.set_ylim(-5, 45)
        self.ax_delta.grid(True, alpha=0.35)
        self.ax_delta.legend(loc="upper right")

        checkbox_ax = self.fig.add_axes([0.80, 0.68, 0.18, 0.14])
        self.checkbox = CheckButtons(
            checkbox_ax,
            ["Solo ultimos\nsegundos"],
            [self.keep_last_seconds],
        )
        for label in self.checkbox.labels:
            label.set_fontsize(11)
        for box in maybe_iter(getattr(self.checkbox, "rectangles", None)):
            box.set_width(0.06)
            box.set_height(0.06)
        for frame in maybe_iter(getattr(self.checkbox, "_frames", None)):
            try:
                frame.set_sizes([80])
            except AttributeError:
                pass
            try:
                frame.set_markersize(9)
            except AttributeError:
                pass
        for check in maybe_iter(getattr(self.checkbox, "_checks", None)):
            try:
                check.set_sizes([80])
            except AttributeError:
                pass
            try:
                check.set_markersize(9)
            except AttributeError:
                pass
        self.checkbox.on_clicked(self.on_checkbox)

        self.window_label = self.fig.text(
            0.80,
            0.61,
            "Mostrar ultimos X segundos",
            va="bottom",
            ha="left",
            fontsize=10,
        )
        self.textbox_ax = self.fig.add_axes([0.80, 0.55, 0.18, 0.05])
        self.textbox = TextBox(self.textbox_ax, "", initial=str(default_window_s))
        self.textbox.on_submit(self.on_window_submit)
        self.set_window_control_visible(self.keep_last_seconds)

        self.fig.canvas.mpl_connect("close_event", self.on_close)

    def on_checkbox(self, _label: str) -> None:
        self.keep_last_seconds = not self.keep_last_seconds
        self.set_window_control_visible(self.keep_last_seconds)
        self.apply_time_window()

    def set_window_control_visible(self, visible: bool) -> None:
        self.window_label.set_visible(visible)
        self.textbox_ax.set_visible(visible)
        self.textbox.ax.set_visible(visible)
        self.fig.canvas.draw_idle()

    def on_window_submit(self, text: str) -> None:
        try:
            value = float(text)
        except ValueError:
            self.textbox.set_val(f"{self.window_s:g}")
            return

        if value <= 0:
            self.textbox.set_val(f"{self.window_s:g}")
            return

        self.window_s = value
        self.apply_time_window()

    def on_close(self, _event) -> None:
        self.running = False
        self.sock.close()

    def drain_udp(self) -> int:
        count = 0
        while True:
            try:
                packet, _addr = self.sock.recvfrom(8192)
            except BlockingIOError:
                break
            except OSError:
                self.running = False
                break

            parsed = parse_packet(packet)
            if parsed is None:
                continue

            rpm = parsed[0]
            torque = parsed[1]
            delta = parsed[2] if len(parsed) >= 5 else None
            delta_ref = parsed[3] if len(parsed) >= 5 else None
            sim_time = parsed[4] if len(parsed) >= 5 else None
            now = time.monotonic()

            if self.stream_stopped:
                self.samples.clear()
                self.stream_stopped = False

            self.samples.append(
                Sample(
                    t_rx=now,
                    rpm=abs(rpm),
                    torque=abs(torque),
                    delta=delta,
                    delta_ref=delta_ref,
                    sim_time=sim_time,
                )
            )
            self.last_packet_time = now
            count += 1

        return count

    def apply_time_window(self) -> None:
        if not self.keep_last_seconds or not self.samples:
            return

        cutoff = time.monotonic() - self.window_s
        while self.samples and self.samples[0].t_rx < cutoff:
            self.samples.popleft()

    def mark_stream_stopped_if_stale(self) -> None:
        if self.last_packet_time is None:
            return

        if time.monotonic() - self.last_packet_time > self.stale_timeout_s:
            self.stream_stopped = True
            self.last_packet_time = None

    def update(self, _frame):
        if not self.running:
            plt.close(self.fig)
            return (
                self.trace_line,
                self.point,
                self.delta_line,
                self.delta_ref_line,
                self.status_text,
            )

        received = self.drain_udp()
        if received:
            self.apply_time_window()
        else:
            self.mark_stream_stopped_if_stale()

        if not self.samples:
            self.trace_line.set_data([], [])
            self.point.set_offsets([[float("nan"), float("nan")]])
            self.delta_line.set_data([], [])
            self.delta_ref_line.set_data([], [])
            self.status_text.set_text("Esperando UDP...")
            return (
                self.trace_line,
                self.point,
                self.delta_line,
                self.delta_ref_line,
                self.status_text,
            )

        rpm = [sample.rpm for sample in self.samples]
        torque = [sample.torque for sample in self.samples]
        self.trace_line.set_data(rpm, torque)
        self.point.set_offsets([[rpm[-1], torque[-1]]])
        self.update_axis_limits(rpm, torque)
        self.update_delta_plot()

        latest_rpm = rpm[-1]
        latest_torque = torque[-1]
        cont = continuous_limit(latest_rpm)
        peak = peak_limit(latest_rpm)

        if latest_torque <= cont:
            region = "CONTINUO"
        elif latest_torque <= peak:
            region = "PICO"
        else:
            region = "FUERA DE LIMITE"

        mode = f"ultimos {self.window_s:g} s" if self.keep_last_seconds else "traza completa"
        if self.stream_stopped:
            status = "simulacion detenida | traza retenida"
        else:
            status = f"region: {region}"

        self.status_text.set_text(
            f"rpm={latest_rpm:.1f}, Tm={latest_torque:.2f} N.m\n"
            f"{status} | {mode}"
        )

        return (
            self.trace_line,
            self.point,
            self.delta_line,
            self.delta_ref_line,
            self.status_text,
        )

    def update_delta_plot(self) -> None:
        delta_samples = [
            sample
            for sample in self.samples
            if sample.delta is not None and sample.delta_ref is not None
        ]

        if not delta_samples:
            self.delta_line.set_data([], [])
            self.delta_ref_line.set_data([], [])
            return

        if all(sample.sim_time is not None for sample in delta_samples):
            t = [sample.sim_time for sample in delta_samples]
        else:
            t0 = delta_samples[0].t_rx
            t = [sample.t_rx - t0 for sample in delta_samples]

        delta = [sample.delta for sample in delta_samples]
        delta_ref = [sample.delta_ref for sample in delta_samples]

        self.delta_line.set_data(t, delta)
        self.delta_ref_line.set_data(t, delta_ref)
        self.update_delta_axis_limits(t, delta, delta_ref)

    def update_axis_limits(self, rpm: list[float], torque: list[float]) -> None:
        if not rpm or not torque:
            return

        max_rpm = max(max(rpm), self.base_xmax)
        max_torque = max(max(torque), self.base_ymax)

        x_margin = max(25.0, 0.05 * max_rpm)
        y_margin = max(2.0, 0.08 * max_torque)

        desired_xlim = (0.0, max_rpm + x_margin)
        desired_ylim = (0.0, max_torque + y_margin)

        current_xlim = self.ax_motor.get_xlim()
        current_ylim = self.ax_motor.get_ylim()

        if (
            abs(current_xlim[1] - desired_xlim[1]) > 1.0
            or abs(current_ylim[1] - desired_ylim[1]) > 0.25
        ):
            self.ax_motor.set_xlim(*desired_xlim)
            self.ax_motor.set_ylim(*desired_ylim)

    def update_delta_axis_limits(
        self,
        t: list[float],
        delta: list[float | None],
        delta_ref: list[float | None],
    ) -> None:
        if not t:
            return

        y_values = [value for value in delta + delta_ref if value is not None]
        if not y_values:
            return

        t_min = min(t)
        t_max = max(t)
        if t_max <= t_min:
            t_max = t_min + 1.0

        y_min = min(min(y_values), -5.0)
        y_max = max(max(y_values), 45.0)
        y_span = max(y_max - y_min, 1.0)
        t_margin = max(0.05, 0.03 * (t_max - t_min))
        y_margin = max(1.0, 0.08 * y_span)

        desired_xlim = (t_min, t_max + t_margin)
        desired_ylim = (y_min - y_margin, y_max + y_margin)

        current_xlim = self.ax_delta.get_xlim()
        current_ylim = self.ax_delta.get_ylim()

        if (
            abs(current_xlim[0] - desired_xlim[0]) > 0.01
            or abs(current_xlim[1] - desired_xlim[1]) > 0.01
            or abs(current_ylim[0] - desired_ylim[0]) > 0.25
            or abs(current_ylim[1] - desired_ylim[1]) > 0.25
        ):
            self.ax_delta.set_xlim(*desired_xlim)
            self.ax_delta.set_ylim(*desired_ylim)

    def run(self) -> None:
        self.animation = FuncAnimation(
            self.fig,
            self.update,
            interval=30,
            blit=False,
            cache_frame_data=False,
        )
        plt.show()
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Realtime motor torque-speed UDP plotter for Simulink."
    )
    parser.add_argument("--host", default="0.0.0.0", help="UDP bind address.")
    parser.add_argument("--port", type=int, default=5010, help="UDP bind port.")
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=1.0,
        help=(
            "Seconds without UDP before marking the stream as stopped. "
            "The trace is kept and reset only when UDP resumes."
        ),
    )
    parser.add_argument(
        "--window",
        type=float,
        default=5.0,
        help="Default seconds retained when the checkbox is enabled.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=20000,
        help="Safety cap for stored points when full trace mode is enabled.",
    )
    args = parser.parse_args()

    app = UdpMotorCurveApp(
        host=args.host,
        port=args.port,
        stale_timeout_s=args.stale_timeout,
        default_window_s=args.window,
        max_points=args.max_points,
    )
    print(f"Listening for UDP packets on {args.host}:{args.port}")
    app.run()


if __name__ == "__main__":
    main()
