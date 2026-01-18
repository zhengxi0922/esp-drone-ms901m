import socket
import struct
import time
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText


CRTP_PORT_SETPOINT = 0x03
CRTP_CHANNEL = 0x00
CRTP_HEADER_SETPOINT = ((CRTP_PORT_SETPOINT & 0x0F) << 4) | (CRTP_CHANNEL & 0x0F)
UDP_DIAG_PING = b"CFPING"
UDP_DIAG_PONG = b"CFPONG"


def build_udp_packet(payload):
    data = payload
    checksum = sum(data) & 0xFF
    return data + bytes([checksum])

def build_setpoint_packet(roll_deg, pitch_deg, yaw_deg, thrust):
    payload = struct.pack("<fffH", roll_deg, pitch_deg, yaw_deg, thrust)
    return build_udp_packet(bytes([CRTP_HEADER_SETPOINT]) + payload)


class UdpController:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP-Drone UDP Controller")
        self.root.resizable(False, False)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        self.streaming = False
        self.last_send = 0.0

        self.ip_var = tk.StringVar(value="192.168.43.42")
        self.port_var = tk.StringVar(value="2390")
        self.roll_var = tk.StringVar(value="0.0")
        self.pitch_var = tk.StringVar(value="0.0")
        self.yaw_var = tk.StringVar(value="0.0")
        self.rate_var = tk.StringVar(value="50")
        self.thrust_var = tk.IntVar(value=0)

        self._build_ui()

    def _build_ui(self):
        pad = {"padx": 6, "pady": 4}
        main = ttk.Frame(self.root)
        main.grid(row=0, column=0, sticky="nsew")

        conn = ttk.LabelFrame(main, text="Connection")
        conn.grid(row=0, column=0, sticky="ew", **pad)
        ttk.Label(conn, text="IP").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(conn, width=16, textvariable=self.ip_var).grid(row=0, column=1, **pad)
        ttk.Label(conn, text="Port").grid(row=0, column=2, sticky="w", **pad)
        ttk.Entry(conn, width=8, textvariable=self.port_var).grid(row=0, column=3, **pad)
        ttk.Button(conn, text="Ping", command=self.ping_link).grid(row=0, column=4, **pad)
        self.link_status = ttk.Label(conn, text="Link: unknown")
        self.link_status.grid(row=0, column=5, sticky="w", **pad)

        sp = ttk.LabelFrame(main, text="Setpoint (deg, thrust)")
        sp.grid(row=1, column=0, sticky="ew", **pad)
        ttk.Label(sp, text="Roll").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(sp, width=10, textvariable=self.roll_var).grid(row=0, column=1, **pad)
        ttk.Label(sp, text="Pitch").grid(row=0, column=2, sticky="w", **pad)
        ttk.Entry(sp, width=10, textvariable=self.pitch_var).grid(row=0, column=3, **pad)
        ttk.Label(sp, text="Yaw").grid(row=0, column=4, sticky="w", **pad)
        ttk.Entry(sp, width=10, textvariable=self.yaw_var).grid(row=0, column=5, **pad)

        ttk.Label(sp, text="Thrust").grid(row=1, column=0, sticky="w", **pad)
        thrust_scale = ttk.Scale(
            sp, from_=0, to=60000, orient="horizontal",
            command=self._on_thrust_scale
        )
        thrust_scale.set(self.thrust_var.get())
        thrust_scale.grid(row=1, column=1, columnspan=4, sticky="ew", **pad)
        self.thrust_entry = ttk.Entry(sp, width=8, textvariable=self.thrust_var)
        self.thrust_entry.grid(row=1, column=5, **pad)

        rate = ttk.LabelFrame(main, text="Streaming")
        rate.grid(row=2, column=0, sticky="ew", **pad)
        ttk.Label(rate, text="Rate (Hz)").grid(row=0, column=0, sticky="w", **pad)
        ttk.Entry(rate, width=8, textvariable=self.rate_var).grid(row=0, column=1, **pad)
        ttk.Button(rate, text="Send Once", command=self.send_once).grid(row=0, column=2, **pad)
        ttk.Button(rate, text="Start Stream", command=self.start_stream).grid(row=0, column=3, **pad)
        ttk.Button(rate, text="Stop Stream", command=self.stop_stream).grid(row=0, column=4, **pad)

        ctrl = ttk.LabelFrame(main, text="Safety")
        ctrl.grid(row=3, column=0, sticky="ew", **pad)
        ttk.Button(ctrl, text="Unlock (thrust=0)", command=self.send_unlock).grid(row=0, column=0, **pad)
        ttk.Button(ctrl, text="E-Stop", command=self.send_estop).grid(row=0, column=1, **pad)
        ttk.Button(ctrl, text="Zero RPY", command=self.zero_rpy).grid(row=0, column=2, **pad)

        log = ttk.LabelFrame(main, text="Log")
        log.grid(row=4, column=0, sticky="ew", **pad)
        self.log_box = ScrolledText(log, width=60, height=8, state="disabled")
        self.log_box.grid(row=0, column=0, **pad)

        note = ("Notes: send thrust=0 once to unlock, then thrust >= 1000.\n"
                "Remove props for testing.")
        ttk.Label(main, text=note).grid(row=5, column=0, sticky="w", **pad)

        sp.columnconfigure(1, weight=1)
        sp.columnconfigure(2, weight=1)
        sp.columnconfigure(3, weight=1)
        sp.columnconfigure(4, weight=1)
        sp.columnconfigure(5, weight=1)

    def _on_thrust_scale(self, value):
        try:
            self.thrust_var.set(int(float(value)))
        except ValueError:
            pass

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{ts}] {msg}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def _get_target(self):
        ip = self.ip_var.get().strip()
        try:
            port = int(self.port_var.get().strip())
        except ValueError:
            raise ValueError("invalid port")
        return ip, port

    def _get_setpoint(self):
        try:
            roll = float(self.roll_var.get().strip())
            pitch = float(self.pitch_var.get().strip())
            yaw = float(self.yaw_var.get().strip())
            thrust = int(self.thrust_var.get())
        except ValueError:
            raise ValueError("invalid setpoint")
        thrust = max(0, min(60000, thrust))
        return roll, pitch, yaw, thrust

    def _send_packet(self, roll, pitch, yaw, thrust):
        ip, port = self._get_target()
        data = build_setpoint_packet(roll, pitch, yaw, thrust)
        self.sock.sendto(data, (ip, port))
        self.last_send = time.time()

    def send_once(self):
        try:
            roll, pitch, yaw, thrust = self._get_setpoint()
            self._send_packet(roll, pitch, yaw, thrust)
            self._log(f"send roll={roll:.2f} pitch={pitch:.2f} yaw={yaw:.2f} thrust={thrust}")
        except Exception as exc:
            messagebox.showerror("Send failed", str(exc))

    def send_unlock(self):
        try:
            roll, pitch, yaw, _ = self._get_setpoint()
            self._send_packet(roll, pitch, yaw, 0)
            self._log("unlock packet sent (thrust=0)")
        except Exception as exc:
            messagebox.showerror("Unlock failed", str(exc))

    def send_estop(self):
        self.stop_stream()
        try:
            self._send_packet(0.0, 0.0, 0.0, 0)
            self._log("e-stop sent")
        except Exception as exc:
            messagebox.showerror("E-stop failed", str(exc))

    def zero_rpy(self):
        self.roll_var.set("0.0")
        self.pitch_var.set("0.0")
        self.yaw_var.set("0.0")
        self._log("roll/pitch/yaw zeroed")

    def start_stream(self):
        if self.streaming:
            return
        self.streaming = True
        self._log("streaming started")
        self._stream_tick()

    def stop_stream(self):
        if self.streaming:
            self.streaming = False
            self._log("streaming stopped")

    def _stream_tick(self):
        if not self.streaming:
            return
        try:
            rate_hz = float(self.rate_var.get().strip())
            if rate_hz <= 0:
                raise ValueError("rate must be > 0")
            roll, pitch, yaw, thrust = self._get_setpoint()
            self._send_packet(roll, pitch, yaw, thrust)
        except Exception as exc:
            self.streaming = False
            messagebox.showerror("Stream error", str(exc))
            return

        interval_ms = max(10, int(1000.0 / rate_hz))
        self.root.after(interval_ms, self._stream_tick)

    def ping_link(self):
        ip, port = self._get_target()
        packet = build_udp_packet(UDP_DIAG_PING)
        try:
            self.sock.sendto(packet, (ip, port))
        except Exception as exc:
            messagebox.showerror("Ping failed", str(exc))
            return

        self.sock.settimeout(0.5)
        try:
            data, _addr = self.sock.recvfrom(128)
        except socket.timeout:
            self.link_status.configure(text="Link: timeout")
            self._log("ping timeout")
            self.sock.setblocking(False)
            return
        except Exception as exc:
            self.link_status.configure(text="Link: error")
            self._log(f"ping error: {exc}")
            self.sock.setblocking(False)
            return
        finally:
            self.sock.setblocking(False)

        if len(data) < 2:
            self.link_status.configure(text="Link: invalid")
            self._log("ping invalid response")
            return

        rx = data[:-1]
        rx_ck = data[-1]
        if (sum(rx) & 0xFF) != rx_ck:
            self.link_status.configure(text="Link: checksum fail")
            self._log("ping checksum mismatch")
            return
        if rx == UDP_DIAG_PONG:
            self.link_status.configure(text="Link: OK")
            self._log("ping ok")
        else:
            self.link_status.configure(text="Link: unexpected")
            self._log("ping unexpected response")


def main():
    root = tk.Tk()
    app = UdpController(root)
    root.mainloop()


if __name__ == "__main__":
    main()
