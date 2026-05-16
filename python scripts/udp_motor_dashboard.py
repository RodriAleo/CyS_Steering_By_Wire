#!/usr/bin/env python3
"""
Realtime torque-speed operating point viewer for Simulink UDP output.

Expected UDP payload:
  - preferred: eight binary doubles
    [rpm_m, Tm, delta_deg, delta_ref_deg, tiempo, Vm, im, Ta],
    little-endian or native endian
  - also accepted: seven binary doubles
    [rpm_m, Tm, delta_deg, delta_ref_deg, tiempo, Vm, im]
  - also accepted: five binary doubles [rpm_m, Tm, delta_deg, delta_ref_deg, tiempo]
  - also accepted: two binary doubles [rpm_m, Tm] for the torque-speed plot only
  - also accepted: text such as "rpm,Tm,delta,delta_ref,time,Vm,im,Ta"

Default Simulink setup:
  UDP Send remote address: 127.0.0.1 if Python runs on the same OS as Simulink.
                           Use the WSL IP if Simulink runs on Windows and this
                           script runs inside WSL.
  UDP Send remote port:    5010
  Signal vector:           [rpm_m; Tm; delta_deg; delta_ref_deg; tiempo; Vm; im; Ta]

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
from matplotlib.gridspec import GridSpec

plt.rcParams["toolbar"] = "None"

VM_LIMIT = 48.0
IM_CONT_LIMIT = 30.0
IM_PEAK_LIMIT = 70.0
TA_BASE_LIMIT = 120.0
TRACE_WINDOW_S = 2.0
UPDATE_INTERVAL_MS = 100


@dataclass
class Sample:
    t_rx: float
    rpm: float
    torque: float
    delta: float | None = None
    delta_ref: float | None = None
    sim_time: float | None = None
    vm: float | None = None
    im: float | None = None
    ta: float | None = None


@dataclass(frozen=True)
class FigureFontSpec:
    base_width: float
    base_height: float
    title_size: float = 10.0
    label_size: float = 13.0
    tick_size: float = 12.0
    legend_size: float = 8.0
    text_size: float = 12.0


def parse_packet(packet: bytes) -> tuple[float, ...] | None:
    """Return common Simulink UDP payload formats."""
    if len(packet) >= 64:
        for fmt in ("<8d", "=8d", ">8d"):
            try:
                values = struct.unpack(fmt, packet[:64])
            except struct.error:
                continue
            if plausible_values(values):
                return values

    if len(packet) >= 56:
        for fmt in ("<7d", "=7d", ">7d"):
            try:
                values = struct.unpack(fmt, packet[:56])
            except struct.error:
                continue
            if plausible_values(values):
                return values

    if len(packet) >= 40:
        for fmt in ("<5d", "=5d", ">5d"):
            try:
                values = struct.unpack(fmt, packet[:40])
            except struct.error:
                continue
            if plausible_values(values):
                return values

    if len(packet) >= 8:
        if len(packet) >= 32:
            for fmt in ("<8f", "=8f", ">8f"):
                try:
                    values = struct.unpack(fmt, packet[:32])
                except struct.error:
                    continue
                if plausible_values(values):
                    return tuple(float(value) for value in values)

        if len(packet) >= 28:
            for fmt in ("<7f", "=7f", ">7f"):
                try:
                    values = struct.unpack(fmt, packet[:28])
                except struct.error:
                    continue
                if plausible_values(values):
                    return tuple(float(value) for value in values)

        if len(packet) >= 20:
            for fmt in ("<5f", "=5f", ">5f"):
                try:
                    values = struct.unpack(fmt, packet[:20])
                except struct.error:
                    continue
                if plausible_values(values):
                    return tuple(float(value) for value in values)

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

    if len(values) >= 8 and plausible_values(values[:8]):
        return tuple(values[:8])

    if len(values) >= 7 and plausible_values(values[:7]):
        return tuple(values[:7])

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
        max_points: int,
    ) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)

        self.stale_timeout_s = stale_timeout_s
        self.window_s = TRACE_WINDOW_S
        self.keep_last_seconds = True
        self.samples: deque[Sample] = deque(maxlen=max_points)
        self.last_packet_time: float | None = None
        self.stream_stopped = False
        self.running = True
        self.figure_font_specs: dict[object, FigureFontSpec] = {}
        self.closing = False

        self.update_interval_ms = UPDATE_INTERVAL_MS

        self.fig, (self.ax_motor, self.ax_delta, self.ax_vm, self.ax_im, self.ax_ta) = self.create_dashboard()
        self.figures = [self.fig]
        self.fig_motor = self.fig

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

        self.ax_motor.set_xlabel("Velocidad motor |rpm|")
        self.ax_motor.set_ylabel("Torque motor |N.m|")
        self.ax_motor.set_title("Curva torque-velocidad")
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
        self.ax_delta.set_xlabel("Tiempo [s]")
        self.ax_delta.set_ylabel("Angulo [deg]")
        self.ax_delta.set_title("Seguimiento de delta")
        self.ax_delta.set_xlim(0, 1)
        self.ax_delta.set_ylim(-5, 45)
        self.ax_delta.grid(True, alpha=0.35)
        self.ax_delta.legend(loc="upper right")

        (self.vm_line,) = self.ax_vm.plot(
            [],
            [],
            color="tab:purple",
            linewidth=1.5,
            label="Vm",
        )
        self.ax_vm.axhline(VM_LIMIT, color="tab:red", linestyle="--", linewidth=1.2, label="+/-48 V")
        self.ax_vm.axhline(-VM_LIMIT, color="tab:red", linestyle="--", linewidth=1.2)
        self.ax_vm.set_xlabel("Tiempo [s]")
        self.ax_vm.set_ylabel("Vm [V]")
        self.ax_vm.set_title("Tension del motor")
        self.ax_vm.set_xlim(0, 1)
        self.ax_vm.set_ylim(-55, 55)
        self.ax_vm.grid(True, alpha=0.35)
        self.ax_vm.legend(loc="upper right")

        (self.im_line,) = self.ax_im.plot(
            [],
            [],
            color="tab:green",
            linewidth=1.5,
            label="im",
        )
        self.ax_im.axhline(IM_PEAK_LIMIT, color="tab:red", linestyle="--", linewidth=1.2, label="+/-70 A pico")
        self.ax_im.axhline(-IM_PEAK_LIMIT, color="tab:red", linestyle="--", linewidth=1.2)
        self.ax_im.axhline(IM_CONT_LIMIT, color="tab:blue", linestyle=":", linewidth=1.2, label="+/-30 A continuo")
        self.ax_im.axhline(-IM_CONT_LIMIT, color="tab:blue", linestyle=":", linewidth=1.2)
        self.ax_im.set_xlabel("Tiempo [s]")
        self.ax_im.set_ylabel("im [A]")
        self.ax_im.set_title("Corriente del motor")
        self.ax_im.set_xlim(0, 1)
        self.ax_im.set_ylim(-80, 80)
        self.ax_im.grid(True, alpha=0.35)
        self.ax_im.legend(loc="upper right")

        (self.ta_line,) = self.ax_ta.plot(
            [],
            [],
            color="tab:red",
            linewidth=1.5,
            label="Ta",
        )
        self.ax_ta.set_xlabel("Tiempo [s]")
        self.ax_ta.set_ylabel("Ta [N.m]")
        self.ax_ta.set_title("Torque autoalineante")
        self.ax_ta.set_xlim(0, 1)
        self.ax_ta.set_ylim(-10, TA_BASE_LIMIT)
        self.ax_ta.grid(True, alpha=0.35)
        self.ax_ta.legend(loc="upper right")

        self.fig.canvas.mpl_connect("close_event", self.on_close)
        self.fig.canvas.mpl_connect("resize_event", self.on_resize)

        self.apply_initial_window_layout()
        self.register_figure_fonts()

    def on_close(self, _event) -> None:
        if self.closing:
            return

        self.closing = True
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        plt.close(self.fig)

    @staticmethod
    def create_dashboard():
        fig = plt.figure(figsize=(6.4, 9.2))
        fig.canvas.manager.set_window_title("Motor dashboard")
        grid = GridSpec(3, 2, figure=fig, height_ratios=[1.15, 1.0, 1.0])
        ax_motor = fig.add_subplot(grid[0, :])
        ax_delta = fig.add_subplot(grid[1, 0])
        ax_ta = fig.add_subplot(grid[1, 1])
        ax_im = fig.add_subplot(grid[2, 0])
        ax_vm = fig.add_subplot(grid[2, 1])
        fig.subplots_adjust(left=0.12, right=0.97, bottom=0.06, top=0.95, hspace=0.52, wspace=0.36)
        return fig, (ax_motor, ax_delta, ax_vm, ax_im, ax_ta)

    def apply_initial_window_layout(self) -> None:
        screen_x, screen_y, screen_w, screen_h = self.get_screen_geometry()
        outer_margin = 12
        titlebar_allowance = 34
        width = min(max(560, screen_w // 3 - 2 * outer_margin), 680)
        height = max(640, screen_h - 2 * outer_margin - titlebar_allowance)
        x = screen_x + screen_w - width - outer_margin
        y = screen_y + outer_margin
        self.set_window_geometry(self.fig, x, y, width, height)

    def get_screen_geometry(self) -> tuple[int, int, int, int]:
        window = self.fig_motor.canvas.manager.window

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
        width = int(width)
        height = int(height)
        fig.set_size_inches(width / fig.dpi, height / fig.dpi, forward=True)

        window = fig.canvas.manager.window
        geometry = f"{width}x{height}+{int(x)}+{int(y)}"

        if hasattr(window, "wm_geometry"):
            try:
                window.wm_geometry(geometry)
                return
            except Exception:
                pass

        if hasattr(window, "setGeometry"):
            try:
                window.setGeometry(int(x), int(y), width, height)
                return
            except Exception:
                pass

        if hasattr(window, "move") and hasattr(window, "resize"):
            try:
                window.move(int(x), int(y))
                window.resize(width, height)
            except Exception:
                pass

    def register_figure_fonts(self) -> None:
        for fig in self.figures:
            width, height = fig.get_size_inches()
            self.figure_font_specs[fig] = FigureFontSpec(width, height)
            self.apply_font_scale(fig)

    def on_resize(self, event) -> None:
        fig = event.canvas.figure
        self.apply_font_scale(fig)
        fig.canvas.draw_idle()

    def apply_font_scale(self, fig) -> None:
        spec = self.figure_font_specs.get(fig)
        if spec is None:
            return

        width, height = fig.get_size_inches()
        scale = min(width / spec.base_width, height / spec.base_height)
        scale = max(0.55, min(1.15, scale))

        for ax in fig.axes:
            ax.title.set_fontsize(spec.title_size * scale)
            ax.xaxis.label.set_fontsize(spec.label_size * scale)
            ax.yaxis.label.set_fontsize(spec.label_size * scale)
            ax.tick_params(labelsize=spec.tick_size * scale)

            legend = ax.get_legend()
            if legend is not None:
                for text in legend.get_texts():
                    text.set_fontsize(spec.legend_size * scale)

        for text in fig.texts:
            text.set_fontsize(spec.text_size * scale)

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
            vm = parsed[5] if len(parsed) >= 7 else None
            im = parsed[6] if len(parsed) >= 7 else None
            ta = parsed[7] if len(parsed) >= 8 else None
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
                    vm=vm,
                    im=im,
                    ta=ta,
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
                self.vm_line,
                self.im_line,
                self.ta_line,
                self.trace_line,
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
            self.vm_line.set_data([], [])
            self.im_line.set_data([], [])
            self.ta_line.set_data([], [])
            return (
                self.trace_line,
                self.point,
                self.delta_line,
                self.delta_ref_line,
                self.vm_line,
                self.im_line,
                self.ta_line,
                self.trace_line,
            )

        rpm = [sample.rpm for sample in self.samples]
        torque = [sample.torque for sample in self.samples]
        self.trace_line.set_data(rpm, torque)
        self.point.set_offsets([[rpm[-1], torque[-1]]])
        self.update_axis_limits(rpm, torque)
        self.update_delta_plot()
        self.update_auxiliary_time_plots()

        return (
            self.trace_line,
            self.point,
            self.delta_line,
            self.delta_ref_line,
            self.vm_line,
            self.im_line,
            self.ta_line,
            self.trace_line,
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

    def update_auxiliary_time_plots(self) -> None:
        electrical_samples = [
            sample
            for sample in self.samples
            if sample.vm is not None and sample.im is not None
        ]

        if not electrical_samples:
            self.vm_line.set_data([], [])
            self.im_line.set_data([], [])
        else:
            t = self.sample_time_axis(electrical_samples)

            vm = [sample.vm for sample in electrical_samples]
            im = [sample.im for sample in electrical_samples]

            self.vm_line.set_data(t, vm)
            self.im_line.set_data(t, im)
            self.update_time_axis_limits(self.ax_vm, t, vm, -VM_LIMIT, VM_LIMIT, margin_min=5.0)
            self.update_time_axis_limits(
                self.ax_im,
                t,
                im,
                -IM_PEAK_LIMIT,
                IM_PEAK_LIMIT,
                margin_min=5.0,
            )

        ta_samples = [sample for sample in self.samples if sample.ta is not None]
        if not ta_samples:
            self.ta_line.set_data([], [])
            return

        t = self.sample_time_axis(ta_samples)
        ta = [sample.ta for sample in ta_samples]
        self.ta_line.set_data(t, ta)
        self.update_time_axis_limits(self.ax_ta, t, ta, -10.0, TA_BASE_LIMIT, margin_min=5.0)

    @staticmethod
    def sample_time_axis(samples: list[Sample]) -> list[float]:
        if all(sample.sim_time is not None for sample in samples):
            return [sample.sim_time for sample in samples if sample.sim_time is not None]

        t0 = samples[0].t_rx
        return [sample.t_rx - t0 for sample in samples]

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

    def update_time_axis_limits(
        self,
        ax,
        t: list[float],
        values: list[float | None],
        base_min: float,
        base_max: float,
        margin_min: float,
    ) -> None:
        y_values = [value for value in values if value is not None]
        if not t or not y_values:
            return

        t_min = min(t)
        t_max = max(t)
        if t_max <= t_min:
            t_max = t_min + 1.0
        elif self.keep_last_seconds:
            t_min = max(t_min, t_max - self.window_s)

        y_min = min(min(y_values), base_min)
        y_max = max(max(y_values), base_max)
        y_span = max(y_max - y_min, 1.0)
        t_margin = max(0.05, 0.03 * (t_max - t_min))
        y_margin = max(margin_min, 0.08 * y_span)

        desired_xlim = (t_min, t_max + t_margin)
        desired_ylim = (y_min - y_margin, y_max + y_margin)

        current_xlim = ax.get_xlim()
        current_ylim = ax.get_ylim()

        if (
            abs(current_xlim[0] - desired_xlim[0]) > 0.01
            or abs(current_xlim[1] - desired_xlim[1]) > 0.01
            or abs(current_ylim[0] - desired_ylim[0]) > 0.25
            or abs(current_ylim[1] - desired_ylim[1]) > 0.25
        ):
            ax.set_xlim(*desired_xlim)
            ax.set_ylim(*desired_ylim)

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
        elif self.keep_last_seconds:
            t_min = max(t_min, t_max - self.window_s)

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
            interval=self.update_interval_ms,
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
        max_points=args.max_points,
    )
    print(f"Listening for UDP packets on {args.host}:{args.port}")
    app.run()


if __name__ == "__main__":
    main()
