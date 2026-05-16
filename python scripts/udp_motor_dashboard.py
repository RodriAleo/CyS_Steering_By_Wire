#!/usr/bin/env python3
"""
Fast realtime dashboard for Simulink UDP output.

Expected preferred UDP payload:
  [rpm_m, Tm, delta_deg, delta_ref_deg, tiempo, Vm, im, Ta]

This version prioritizes staying near realtime:
  - UDP reception runs in a background thread.
  - The plot only draws the latest time window.
  - Curves are decimated before drawing.
  - Axes use fixed y-limits to avoid expensive autoscaling.
"""

from __future__ import annotations

import argparse
import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import FormatStrFormatter

plt.rcParams["toolbar"] = "None"

VM_LIMIT = 48.0
IM_CONT_LIMIT = 30.0
IM_PEAK_LIMIT = 70.0
TA_LIMIT = 120.0

U130_RPM = [0.0, 400.0, 800.0]
U130_T_CONT = [18.0, 18.0, 0.0]
U130_T_PEAK = [40.0, 40.0, 0.0]

PACKET_8D_LE = struct.Struct("<8d")
PACKET_7D_LE = struct.Struct("<7d")
PACKET_5D_LE = struct.Struct("<5d")
PACKET_2D_LE = struct.Struct("<2d")


@dataclass(slots=True)
class Sample:
    rx_time: float
    rpm: float
    torque: float
    delta: float | None = None
    delta_ref: float | None = None
    sim_time: float | None = None
    vm: float | None = None
    im: float | None = None
    ta: float | None = None


def plausible(values: tuple[float, ...]) -> bool:
    return all(value == value and abs(value) < 1.0e6 for value in values)


def parse_packet(packet: bytes) -> Sample | None:
    values: tuple[float, ...] | None = None

    # Fast path for the current Simulink configuration.
    if len(packet) >= PACKET_8D_LE.size:
        values = PACKET_8D_LE.unpack(packet[: PACKET_8D_LE.size])
        if not plausible(values):
            values = None

    if values is None and len(packet) >= PACKET_7D_LE.size:
        values = PACKET_7D_LE.unpack(packet[: PACKET_7D_LE.size])
        if not plausible(values):
            values = None

    if values is None and len(packet) >= PACKET_5D_LE.size:
        values = PACKET_5D_LE.unpack(packet[: PACKET_5D_LE.size])
        if not plausible(values):
            values = None

    if values is None and len(packet) >= PACKET_2D_LE.size:
        values = PACKET_2D_LE.unpack(packet[: PACKET_2D_LE.size])
        if not plausible(values):
            values = None

    if values is None:
        try:
            text = packet.decode("utf-8", errors="ignore").strip()
            for sep in (",", ";", "\t"):
                text = text.replace(sep, " ")
            parsed = tuple(float(part) for part in text.split())
        except ValueError:
            return None
        if len(parsed) < 2 or not plausible(parsed):
            return None
        values = parsed

    now = time.monotonic()
    return Sample(
        rx_time=now,
        rpm=abs(values[0]),
        torque=abs(values[1]),
        delta=values[2] if len(values) >= 5 else None,
        delta_ref=values[3] if len(values) >= 5 else None,
        sim_time=values[4] if len(values) >= 5 else None,
        vm=values[5] if len(values) >= 7 else None,
        im=values[6] if len(values) >= 7 else None,
        ta=values[7] if len(values) >= 8 else None,
    )


def decimate(x: list[float], *ys: list[float | None], max_points: int) -> tuple[list[float], ...]:
    if len(x) <= max_points:
        return (x, *ys)

    step = max(1, len(x) // max_points)
    indexes = list(range(0, len(x), step))
    if indexes[-1] != len(x) - 1:
        indexes.append(len(x) - 1)

    result: list[list[float]] = [[x[i] for i in indexes]]
    for y in ys:
        result.append([y[i] for i in indexes])
    return tuple(result)


class FastUdpDashboard:
    def __init__(
        self,
        host: str,
        port: int,
        window_s: float,
        update_ms: int,
        stale_timeout_s: float,
        max_samples: int,
        max_plot_points: int,
    ) -> None:
        self.host = host
        self.port = port
        self.window_s = window_s
        self.update_ms = update_ms
        self.stale_timeout_s = stale_timeout_s
        self.max_plot_points = max_plot_points

        self.samples: deque[Sample] = deque(maxlen=max_samples)
        self.lock = threading.Lock()
        self.running = True
        self.last_packet_rx: float | None = None
        self.last_sim_time: float | None = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(0.05)

        self.thread = threading.Thread(target=self.receive_loop, name="udp-receiver", daemon=True)
        self.thread.start()

        self.fig = plt.figure(figsize=(6.4, 9.2))
        self.set_window_title(self.fig, "UDP Dashboard Fast")
        grid = GridSpec(3, 2, figure=self.fig, height_ratios=[1.15, 1.0, 1.0])
        self.ax_motor = self.fig.add_subplot(grid[0, :])
        self.ax_delta = self.fig.add_subplot(grid[1, 0])
        self.ax_ta = self.fig.add_subplot(grid[1, 1])
        self.ax_im = self.fig.add_subplot(grid[2, 0])
        self.ax_vm = self.fig.add_subplot(grid[2, 1])
        self.fig.subplots_adjust(left=0.12, right=0.97, bottom=0.06, top=0.95, hspace=0.52, wspace=0.36)

        self.setup_axes()
        self.apply_initial_window_layout()
        self.fig.canvas.mpl_connect("close_event", self.on_close)

    def setup_axes(self) -> None:
        self.ax_motor.plot(U130_RPM, U130_T_CONT, color="tab:blue", linewidth=2.0, label="Limite continuo")
        self.ax_motor.plot(U130_RPM, U130_T_PEAK, color="tab:red", linestyle="--", linewidth=2.0, label="Limite pico")
        (self.motor_trace,) = self.ax_motor.plot([], [], color="black", linewidth=1.0, label="Trayectoria")
        self.motor_point = self.ax_motor.scatter([], [], s=36, color="black", zorder=4)
        self.ax_motor.set_title("Curva torque-velocidad", fontsize=10)
        self.ax_motor.set_xlabel("Velocidad motor |rpm|")
        self.ax_motor.set_ylabel("Torque motor |N.m|")
        self.ax_motor.set_xlim(0, 850)
        self.ax_motor.set_ylim(0, 45)
        self.ax_motor.grid(True, alpha=0.35)
        self.ax_motor.legend(loc="upper right", fontsize=8)

        (self.delta_line,) = self.ax_delta.plot([], [], color="tab:blue", linewidth=1.2, label="delta")
        (self.delta_ref_line,) = self.ax_delta.plot([], [], color="tab:orange", linewidth=1.2, label="delta_ref")
        self.setup_time_axis(self.ax_delta, "Seguimiento de delta", "Angulo [deg]", -45, 45)
        self.ax_delta.legend(loc="upper right", fontsize=8)

        (self.ta_line,) = self.ax_ta.plot([], [], color="tab:red", linewidth=1.2, label="Ta")
        self.setup_time_axis(self.ax_ta, "Torque autoalineante", "Ta [N.m]", -TA_LIMIT, TA_LIMIT)
        self.ax_ta.legend(loc="upper right", fontsize=8)

        (self.im_line,) = self.ax_im.plot([], [], color="tab:green", linewidth=1.2, label="im")
        self.ax_im.axhline(IM_PEAK_LIMIT, color="tab:red", linestyle="--", linewidth=1.0, label="+/-70 A")
        self.ax_im.axhline(-IM_PEAK_LIMIT, color="tab:red", linestyle="--", linewidth=1.0)
        self.ax_im.axhline(IM_CONT_LIMIT, color="tab:blue", linestyle=":", linewidth=1.0, label="+/-30 A")
        self.ax_im.axhline(-IM_CONT_LIMIT, color="tab:blue", linestyle=":", linewidth=1.0)
        self.setup_time_axis(self.ax_im, "Corriente del motor", "im [A]", -80, 80)
        self.ax_im.legend(loc="upper right", fontsize=8)

        (self.vm_line,) = self.ax_vm.plot([], [], color="tab:purple", linewidth=1.2, label="Vm")
        self.ax_vm.axhline(VM_LIMIT, color="tab:red", linestyle="--", linewidth=1.0, label="+/-48 V")
        self.ax_vm.axhline(-VM_LIMIT, color="tab:red", linestyle="--", linewidth=1.0)
        self.setup_time_axis(self.ax_vm, "Tension del motor", "Vm [V]", -55, 55)
        self.ax_vm.legend(loc="upper right", fontsize=8)

    @staticmethod
    def setup_time_axis(ax, title: str, ylabel: str, ymin: float, ymax: float) -> None:
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("Tiempo [s]")
        ax.set_ylabel(ylabel)
        ax.set_xlim(0, 2)
        ax.set_ylim(ymin, ymax)
        ax.xaxis.set_major_formatter(FormatStrFormatter("%.1f"))
        ax.grid(True, alpha=0.35)

    def receive_loop(self) -> None:
        while self.running:
            try:
                packet, _addr = self.sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break

            sample = parse_packet(packet)
            if sample is None:
                continue

            with self.lock:
                if (
                    self.last_sim_time is not None
                    and sample.sim_time is not None
                    and sample.sim_time < self.last_sim_time - 0.05
                ):
                    self.samples.clear()

                self.samples.append(sample)
                self.last_packet_rx = sample.rx_time
                if sample.sim_time is not None:
                    self.last_sim_time = sample.sim_time

                self.drop_old_locked(sample)

    def drop_old_locked(self, latest: Sample) -> None:
        if not self.samples:
            return

        margin = 0.25
        if latest.sim_time is not None:
            cutoff = latest.sim_time - self.window_s - margin
            while self.samples and self.samples[0].sim_time is not None and self.samples[0].sim_time < cutoff:
                self.samples.popleft()
        else:
            cutoff = latest.rx_time - self.window_s - margin
            while self.samples and self.samples[0].rx_time < cutoff:
                self.samples.popleft()

    def snapshot(self) -> list[Sample]:
        with self.lock:
            if not self.samples:
                return []
            latest = self.samples[-1]
            self.drop_old_locked(latest)
            return list(self.samples)

    def update(self, _frame):
        samples = self.snapshot()
        if not samples:
            return self.artists()

        latest = samples[-1]
        use_sim_time = latest.sim_time is not None
        t_latest = latest.sim_time if use_sim_time else latest.rx_time
        assert t_latest is not None
        t_min = max(0.0 if use_sim_time else samples[0].rx_time, t_latest - self.window_s)
        t_max = max(t_min + 0.01, t_latest)

        time_samples = [sample for sample in samples if (sample.sim_time is not None if use_sim_time else True)]
        t = [(sample.sim_time if use_sim_time else sample.rx_time) for sample in time_samples]
        t = [value for value in t if value is not None]
        if not t:
            return self.artists()

        if not use_sim_time:
            t0 = t[-1] - self.window_s
            t_plot = [value - t0 for value in t]
            xlim = (0, self.window_s)
        else:
            t_plot = t
            xlim = (t_min, t_max)

        rpm = [sample.rpm for sample in time_samples]
        torque = [sample.torque for sample in time_samples]
        rpm_d, torque_d = decimate(rpm, torque, max_points=self.max_plot_points)
        self.motor_trace.set_data(rpm_d, torque_d)
        self.motor_point.set_offsets([[latest.rpm, latest.torque]])

        self.update_time_line(
            self.ax_delta,
            xlim,
            t_plot,
            time_samples,
            (self.delta_line, "delta"),
            (self.delta_ref_line, "delta_ref"),
        )
        self.update_time_line(self.ax_ta, xlim, t_plot, time_samples, (self.ta_line, "ta"))
        self.update_time_line(self.ax_im, xlim, t_plot, time_samples, (self.im_line, "im"))
        self.update_time_line(self.ax_vm, xlim, t_plot, time_samples, (self.vm_line, "vm"))

        return self.artists()

    def update_time_line(self, ax, xlim, t: list[float], samples: list[Sample], *series) -> None:
        ax.set_xlim(*xlim)
        for line, attr in series:
            values = [getattr(sample, attr) for sample in samples]
            valid = [(x, y) for x, y in zip(t, values) if y is not None]
            if not valid:
                line.set_data([], [])
                continue
            x_values, y_values = zip(*valid)
            x_d, y_d = decimate(list(x_values), list(y_values), max_points=self.max_plot_points)
            line.set_data(x_d, y_d)

    def artists(self):
        return (
            self.motor_trace,
            self.motor_point,
            self.delta_line,
            self.delta_ref_line,
            self.ta_line,
            self.im_line,
            self.vm_line,
        )

    def on_close(self, _event) -> None:
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass

    def apply_initial_window_layout(self) -> None:
        screen_x, screen_y, screen_w, screen_h = self.get_screen_geometry()
        margin = 12
        width = min(max(560, screen_w // 3 - 2 * margin), 680)
        height = max(640, screen_h - 2 * margin - 34)
        x = screen_x + screen_w - width - margin
        y = screen_y + margin
        self.set_window_geometry(self.fig, x, y, width, height)

    def get_screen_geometry(self) -> tuple[int, int, int, int]:
        window = self.fig.canvas.manager.window
        if hasattr(window, "screen"):
            try:
                geometry = window.screen().availableGeometry()
                return geometry.x(), geometry.y(), geometry.width(), geometry.height()
            except Exception:
                pass
        if hasattr(window, "winfo_screenwidth"):
            try:
                return 0, 0, window.winfo_screenwidth(), window.winfo_screenheight()
            except Exception:
                pass
        return 0, 0, 1920, 1080

    def set_window_geometry(self, fig, x: int, y: int, width: int, height: int) -> None:
        fig.set_size_inches(width / fig.dpi, height / fig.dpi, forward=True)
        window = fig.canvas.manager.window
        geometry = f"{int(width)}x{int(height)}+{int(x)}+{int(y)}"
        if hasattr(window, "wm_geometry"):
            try:
                window.wm_geometry(geometry)
                return
            except Exception:
                pass
        if hasattr(window, "setGeometry"):
            try:
                window.setGeometry(int(x), int(y), int(width), int(height))
            except Exception:
                pass

    @staticmethod
    def set_window_title(fig, title: str) -> None:
        manager = fig.canvas.manager
        manager.set_window_title(title)
        window = getattr(manager, "window", None)
        if window is None:
            return
        for method_name in ("title", "wm_title", "iconname", "wm_iconname"):
            method = getattr(window, method_name, None)
            if method is None:
                continue
            try:
                method(title)
            except Exception:
                pass
        if hasattr(window, "setWindowTitle"):
            try:
                window.setWindowTitle(title)
            except Exception:
                pass

    def run(self) -> None:
        self.animation = FuncAnimation(
            self.fig,
            self.update,
            interval=self.update_ms,
            blit=False,
            cache_frame_data=False,
        )
        plt.show()
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        self.thread.join(timeout=0.5)


def main() -> None:
    parser = argparse.ArgumentParser(description="Fast UDP dashboard for Simulink motor signals.")
    parser.add_argument("--host", default="0.0.0.0", help="UDP bind address.")
    parser.add_argument("--port", type=int, default=5010, help="UDP bind port.")
    parser.add_argument("--window", type=float, default=2.0, help="Time window shown in seconds.")
    parser.add_argument("--update-ms", type=int, default=50, help="Plot refresh interval in ms.")
    parser.add_argument("--stale-timeout", type=float, default=1.0, help="Seconds without UDP before stale.")
    parser.add_argument("--max-samples", type=int, default=100000, help="Stored sample cap.")
    parser.add_argument("--max-plot-points", type=int, default=350, help="Max points drawn per line.")
    args = parser.parse_args()

    app = FastUdpDashboard(
        host=args.host,
        port=args.port,
        window_s=args.window,
        update_ms=args.update_ms,
        stale_timeout_s=args.stale_timeout,
        max_samples=args.max_samples,
        max_plot_points=args.max_plot_points,
    )
    print(f"Listening for UDP packets on {args.host}:{args.port}")
    app.run()


if __name__ == "__main__":
    main()
