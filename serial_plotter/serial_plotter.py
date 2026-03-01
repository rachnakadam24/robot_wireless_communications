#!/usr/bin/env python3
"""
Serial Monitor - Split pane with Sequence/Macro Support
Fixed: Added safety checks for sequence interval input to prevent TclError.
"""

import tkinter as tk
from tkinter import ttk, font
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime

# ─── Theme ────────────────────────────────────────────────────────────────────
BG          = "#0d1117"
BG_PANEL    = "#161b22"
BG_INPUT    = "#21262d"
BORDER      = "#30363d"
RX_COLOR    = "#39d353"   # green
TX_COLOR    = "#58a6ff"   # blue
TS_COLOR    = "#484f58"   # grey
ERR_COLOR   = "#f85149"   # red
FG          = "#e6edf3"
ACCENT      = "#238636"
ACCENT_HOV  = "#2ea043"

BAUDS = ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]

class SerialMonitor(tk.Tk):
    def __init__(self):
        super().__init__()

        # Resolve fonts
        families = font.families()
        self.mono_font = ("Cascadia Code", 10) if "Cascadia Code" in families else ("Courier New", 10)
        self.ui_font = ("Segoe UI", 10) if "Segoe UI" in families else ("Helvetica", 10)

        self.title("Serial Monitor Pro")
        self.configure(bg=BG)
        self.geometry("1400x800")

        self.serial = None
        self._running = False
        self.line_ending = tk.StringVar(value="\\n")
        self.show_ts = tk.BooleanVar(value=True)
        self.autoscroll = tk.BooleanVar(value=True)
        
        # Sequence Variables
        self.seq_running = False
        self.seq_loop = tk.BooleanVar(value=False)
        self.seq_interval_var = tk.StringVar(value="1000") # Use StringVar for safer validation
        self.seq_index = 0

        self._build_ui()
        self._refresh_ports()

    def _build_ui(self):
        # --- Top Bar ---
        topbar = tk.Frame(self, bg=BG_PANEL, height=50)
        topbar.pack(fill="x", side="top")
        
        inner = tk.Frame(topbar, bg=BG_PANEL)
        inner.pack(side="left", padx=10)

        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(inner, textvariable=self.port_var, width=15, state="readonly")
        self.port_cb.pack(side="left", padx=5)

        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(inner, textvariable=self.baud_var, values=BAUDS, width=10, state="readonly").pack(side="left", padx=5)

        self.conn_btn = tk.Button(inner, text="CONNECT", command=self._toggle_connect, bg=ACCENT, fg="white", relief="flat", padx=10)
        self.conn_btn.pack(side="left", padx=10)

        # --- Main Layout (Paned) ---
        main_panes = tk.PanedWindow(self, orient="horizontal", bg=BORDER, sashwidth=4)
        main_panes.pack(fill="both", expand=True, padx=10, pady=5)

        # Left: RX Log
        rx_frame = tk.Frame(main_panes, bg=BG_PANEL)
        tk.Label(rx_frame, text="RECEIVE", bg=BG_PANEL, fg=RX_COLOR, font=(self.ui_font[0], 9, "bold")).pack(anchor="w", padx=5)
        self.rx_text = self._make_log(rx_frame)
        self.rx_text.tag_config("rx", foreground=RX_COLOR)
        self.rx_text.tag_config("ts", foreground=TS_COLOR)
        main_panes.add(rx_frame, stretch="always")

        # Center: TX Log & Manual Input
        tx_frame = tk.Frame(main_panes, bg=BG_PANEL)
        tk.Label(tx_frame, text="SEND / HISTORY", bg=BG_PANEL, fg=TX_COLOR, font=(self.ui_font[0], 9, "bold")).pack(anchor="w", padx=5)
        self.tx_text = self._make_log(tx_frame)
        self.tx_text.tag_config("tx", foreground=TX_COLOR)
        
        input_bar = tk.Frame(tx_frame, bg=BG_INPUT)
        input_bar.pack(fill="x", side="bottom", padx=5, pady=5)
        self.entry_var = tk.StringVar()
        self.entry = tk.Entry(input_bar, textvariable=self.entry_var, bg=BG_INPUT, fg=FG, insertbackground=TX_COLOR, relief="flat")
        self.entry.pack(side="left", fill="x", expand=True, padx=5, ipady=5)
        self.entry.bind("<Return>", lambda e: self._send_manual())
        main_panes.add(tx_frame, stretch="always")

        # Right: Sequence Controller
        seq_frame = tk.Frame(main_panes, bg=BG_PANEL, width=350)
        tk.Label(seq_frame, text="SEQUENCE CONTROLLER", bg=BG_PANEL, fg=FG, font=(self.ui_font[0], 9, "bold")).pack(pady=5)
        
        # Control Buttons
        ctrl_btns = tk.Frame(seq_frame, bg=BG_PANEL)
        ctrl_btns.pack(fill="x", padx=10)
        
        self.seq_btn = tk.Button(ctrl_btns, text="▶ START SEQ", command=self._toggle_sequence, bg="#238636", fg="white", relief="flat")
        self.seq_btn.pack(side="left", expand=True, fill="x", padx=2)
        
        tk.Button(ctrl_btns, text="CLEAR", command=lambda: self.seq_editor.delete("1.0", "end"), bg=BG_INPUT, fg=FG, relief="flat").pack(side="left", padx=2)

        # Sequence Settings
        settings = tk.Frame(seq_frame, bg=BG_PANEL)
        settings.pack(fill="x", padx=10, pady=10)
        
        tk.Label(settings, text="Interval (ms):", bg=BG_PANEL, fg=TS_COLOR).pack(side="left")
        # Changed to StringVar validation for robustness
        tk.Entry(settings, textvariable=self.seq_interval_var, width=6, bg=BG_INPUT, fg=FG, relief="flat").pack(side="left", padx=5)
        
        tk.Checkbutton(settings, text="Loop", variable=self.seq_loop, bg=BG_PANEL, fg=FG, selectcolor=BG_INPUT).pack(side="left", padx=10)

        # Editor
        self.seq_editor = tk.Text(seq_frame, bg=BG_INPUT, fg=FG, font=self.mono_font, insertbackground=FG, relief="flat", height=20)
        self.seq_editor.pack(fill="both", expand=True, padx=10, pady=5)
        self.seq_editor.tag_config("active", background="#21262d", foreground=RX_COLOR)
        
        # Placeholder text
        self.seq_editor.insert("1.0", '{"id":2,"left":90,"right":90,"gripper":"open"}\n{"id":2,"left":90,"right":90,"gripper":"close"}')
        
        main_panes.add(seq_frame, stretch="never")

        # Status Bar
        self.status_lbl = tk.Label(self, text="Disconnected", bg=BG_PANEL, fg=TS_COLOR)
        self.status_lbl.pack(side="bottom", fill="x")

    def _make_log(self, parent):
        f = tk.Frame(parent, bg=BG_PANEL)
        f.pack(fill="both", expand=True, padx=5, pady=5)
        t = tk.Text(f, bg=BG_PANEL, fg=FG, font=self.mono_font, relief="flat", state="disabled", wrap="word")
        t.pack(fill="both", expand=True)
        return t

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports: self.port_cb.current(0)

    def _toggle_connect(self):
        if self._running: self._disconnect()
        else: self._connect()

    def _connect(self):
        try:
            self.serial = serial.Serial(self.port_var.get(), int(self.baud_var.get()), timeout=0.1)
            self._running = True
            threading.Thread(target=self._rx_loop, daemon=True).start()
            self.conn_btn.config(text="DISCONNECT", bg=ERR_COLOR)
            self.status_lbl.config(text=f"Connected to {self.port_var.get()}")
        except Exception as e:
            self._log_rx(f"Error: {e}", "err")

    def _disconnect(self):
        self._running = False
        self.seq_running = False
        if self.serial: self.serial.close()
        self.conn_btn.config(text="CONNECT", bg=ACCENT)
        self.status_lbl.config(text="Disconnected")

    def _rx_loop(self):
        while self._running:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='replace').strip()
                    if line: self.after(0, self._log_rx, line)
            except: break

    def _send_manual(self):
        msg = self.entry_var.get()
        if msg:
            self._send_raw(msg)
            self.entry_var.set("")

    def _send_raw(self, msg):
        if not self.serial or not self.serial.is_open: return
        le_map = {"None": "", "\\n": "\n", "\\r": "\r", "\\r\\n": "\r\n"}
        full_msg = msg + le_map.get(self.line_ending.get(), "\n")
        try:
            self.serial.write(full_msg.encode('utf-8'))
            self._log_tx(msg)
        except Exception as e:
            self._log_rx(f"TX Error: {e}", "ts")

    def _toggle_sequence(self):
        if self.seq_running:
            self.seq_running = False
            self.seq_btn.config(text="▶ START SEQ", bg=ACCENT)
        else:
            if not self._running:
                self._log_rx("Connect to Serial first!", "ts")
                return
            self.seq_running = True
            self.seq_index = 0
            self.seq_btn.config(text="■ STOP SEQ", bg=ERR_COLOR)
            self._run_sequence_step()

    def _get_safe_interval(self):
        """Helper to return an integer interval even if input is invalid."""
        try:
            val = int(self.seq_interval_var.get())
            return max(1, val) # Minimum 1ms
        except ValueError:
            return 1000 # Default to 1s if box is empty or not a number

    def _run_sequence_step(self):
        if not self.seq_running or not self._running:
            return

        # Get all lines
        lines = self.seq_editor.get("1.0", "end-1c").splitlines()
        lines = [l.strip() for l in lines if l.strip()]

        if not lines:
            self._toggle_sequence()
            return

        if self.seq_index >= len(lines):
            if self.seq_loop.get():
                self.seq_index = 0
            else:
                self._toggle_sequence()
                return

        # Visual feedback in editor
        self.seq_editor.tag_remove("active", "1.0", "end")
        start_idx = f"{self.seq_index + 1}.0"
        end_idx = f"{self.seq_index + 1}.end"
        self.seq_editor.tag_add("active", start_idx, end_idx)
        self.seq_editor.see(start_idx)

        # Send command
        cmd = lines[self.seq_index]
        self._send_raw(cmd)
        
        self.seq_index += 1
        
        # Schedule next step with safety check
        interval = self._get_safe_interval()
        self.after(interval, self._run_sequence_step)

    def _log_rx(self, text, tag="rx"):
        self._append(self.rx_text, text, tag)

    def _log_tx(self, text, tag="tx"):
        self._append(self.tx_text, text, tag)

    def _append(self, widget, text, tag):
        widget.config(state="normal")
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        widget.insert("end", f"[{ts}] ", "ts")
        widget.insert("end", text + "\n", tag)
        widget.config(state="disabled")
        if self.autoscroll.get(): widget.see("end")

    def on_close(self):
        self._running = False
        if self.serial: self.serial.close()
        self.destroy()

if __name__ == "__main__":
    app = SerialMonitor()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()