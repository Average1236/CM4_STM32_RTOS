"""
dm_h3510_gui.py
Qt6 (PySide6) + python-can demo for DM-H3510 wheel motor (CAN @ slcan)

Features:
 - Connect to slcan device (serial) via python-can (interface='slcan')
 - Send commands: enable / disable / save zero / clear / store params
 - Send control frames: MIT (bitpacked), position/velocity (float32 little-endian), force-position (p/v/i)
 - Read/write registers (ID 0x7FF + cmd 0x33/0x55)
 - Parse & display feedback frames (raw + best-effort parsed fields)
 - Parse & display register read/write/store responses
 - Background thread for receiving CAN messages -> Qt signal
Notes: mapping / scaling assumptions are documented inline. Verify with device manual.
Reference: DM-H3510 user manual (provided by user).
"""

import struct
import sys
import threading
import time
import csv
import os
from typing import Optional

from PySide6.QtCore import Qt, QThread, Signal, Slot, QTimer
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QTextEdit, QComboBox, QSpinBox, QMessageBox, QGridLayout
)

import can


# ----------------------------
# Utility packing / parsing
# ----------------------------

def float_to_le_bytes(f: float) -> bytes:
    return struct.pack('<f', float(f))


def le_bytes_to_float(b: bytes) -> float:
    return struct.unpack('<f', b)[0]


def pack_enable() -> bytes:
    return bytes([0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC])


def pack_disable() -> bytes:
    return bytes([0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD])


def pack_save_zero() -> bytes:
    return bytes([0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE])


def pack_clear_error() -> bytes:
    return bytes([0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB])


def pack_store_params() -> bytes:
    # As doc: ID=0x7FF, D0..D3 = 0xAA 0x01 ; but when sending as control frame (not via 0x7FF),
    # doc shows storage command uses message with D[0]=0xAA D[1]=0x01 and same STD ID = 0x7FF.
    # We'll send via 0x7FF with four bytes.
    return bytes([0xAA, 0x01, 0x00, 0x00])  # the code will send with arbitration_id=0x7FF


def pack_read_register(canid_low: int, canid_high: int, rid: int) -> bytes:
    # Per spec: frame ID 0x7FF, D[0]=STD CANID_L, D[1]=CANID_H, D[2]=0x33, D[3]=RID
    return bytes([canid_low & 0xFF, canid_high & 0xFF, 0x33, rid & 0xFF])


def pack_write_register(canid_low: int, canid_high: int, rid: int, data4: bytes) -> bytes:
    # D0..D7: 0x7FF STD CANID_L CANID_H 0x55 RID DATA (4 bytes)
    d = bytes([canid_low & 0xFF, canid_high & 0xFF, 0x55, rid & 0xFF])
    if len(data4) != 4:
        raise ValueError("Data must be 4 bytes")
    return d + data4


# ----------------------------
# Utility packing / parsing
# ----------------------------

def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """
    Converts a float to an unsigned int, given range and number of bits.
    As defined in the DAMIAO manual V1.4, page 34.
    """
    span = x_max - x_min
    if span == 0:
        return 0
    offset = x_min
    # Clamp the input to the valid range to prevent out-of-bounds values
    x_clamped = max(x_min, min(x, x_max))
    return int((x_clamped - offset) * ((1 << bits) - 1) / span)


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    """
    Converts an unsigned int back to float, given range and number of bits.
    Inverse mapping of float_to_uint() used by MIT mode packing.
    """
    span = x_max - x_min
    if span == 0:
        return float(x_min)
    x_int_clamped = max(0, min(int(x_int), (1 << bits) - 1))
    return float(x_int_clamped) * span / ((1 << bits) - 1) + x_min

def pack_mit_frame(p_des: float, v_des: float, kp: float, kd: float, t_ff: float,
                   pmin=-12.5, pmax=12.5,
                   vmin=-45.0, vmax=45.0,
                   kpmin=0.0, kpmax=500.0,
                   kdmin=0.0, kdmax=5.0,
                   tmin=-18.0, tmax=18.0):
    """
    Pack MIT mode control frame as per DAMIAO manual V1.4, page 33 & 34.
    
    The mapping ranges (pmin/pmax, etc.) should match the driver's settings.
    Default values are from the manual's example.
    """
    # Convert floats to unsigned integers using the specified ranges
    pos_tmp = float_to_uint(p_des, pmin, pmax, 16)
    vel_tmp = float_to_uint(v_des, vmin, vmax, 12)
    kp_tmp  = float_to_uint(kp,  kpmin, kpmax, 12)
    kd_tmp  = float_to_uint(kd,  kdmin, kdmax, 12)
    tor_tmp = float_to_uint(t_ff, tmin, tmax, 12)

    # Pack into 8 bytes according to the bit layout on page 33
    data = bytearray(8)
    data[0] = (pos_tmp >> 8) & 0xFF          # P[15:8]
    data[1] = pos_tmp & 0xFF                 # P[7:0]
    data[2] = (vel_tmp >> 4) & 0xFF          # V[11:4]
    data[3] = ((vel_tmp & 0x0F) << 4) | ((kp_tmp >> 8) & 0x0F)  # V[3:0] | Kp[11:8]
    data[4] = kp_tmp & 0xFF                  # Kp[7:0]
    data[5] = (kd_tmp >> 4) & 0xFF           # Kd[11:4]
    data[6] = ((kd_tmp & 0x0F) << 4) | ((tor_tmp >> 8) & 0x0F)  # Kd[3:0] | T[11:8]
    data[7] = tor_tmp & 0xFF                 # T[7:0]

    return bytes(data)


def pack_force_position_frame(p_des: float, v_des: float, i_des: float) -> bytes:
    """
    Mode 4 (Force-Position Mixed) packing
    - p_des: 32-bit float, little-endian (bytes 0-3)
    - v_des: Uint16 * 100, range 0-10000 (bytes 4-5)
    - i_des: Uint16 * 10000, range 0-10000 (bytes 6-7)
    """
    # p_des
    b_p = struct.pack('<f', float(p_des))
    
    # v_des (scaled by 100, unsigned)
    v_scaled = float(v_des) * 100.0
    v_clamped = max(0, min(int(v_scaled), 10000))
    b_v = struct.pack('<H', v_clamped)
    
    # i_des (scaled by 10000, unsigned)
    i_scaled = float(i_des) * 10000.0
    i_clamped = max(0, min(int(i_scaled), 10000))
    b_i = struct.pack('<H', i_clamped)
    
    return b_p + b_v + b_i



def parse_feedback_frame(data: bytes,
                         pmin: float = -12.5, pmax: float = 12.5,
                         vmin: float = -45.0, vmax: float = 45.0,
                         tmin: float = -18.0, tmax: float = 18.0):
    """
    Parse the feedback frame as per DAMIAO manual V1.4, page 32.
    
    Frame format:
    ID: MST_ID
    D[0]: ID | (ERR << 4)
    D[1]: POS[15:8]
    D[2]: POS[7:0]
    D[3]: VEL[11:4]
    D[4]: VEL[3:0] | T[11:8]
    D[5]: T[7:0]
    D[6]: T_MOS[7:0]
    D[7]: T_Rotor[7:0]
    """
    if len(data) < 8:
        return {'raw': data.hex(), 'error': 'short_frame'}

    D = list(data)
    out = {'raw': data.hex(), 'D': D}

    # out['mst_id'] = D[0]

    # D[0]: Low 4 bits are Controller ID, High 4 bits are Error/Status
    out['controller_id'] = D[0] & 0x0F
    out['err_code'] = (D[0] >> 4) & 0x0F
    ERR_MAP = {
        0: "Disabled",
        1: "Enabled",
        8: "Over Voltage",
        9: "Under Voltage",
        10: "Over Current",
        11: "MOS Over Temp",
        12: "Coil Over Temp",
        13: "Comm Timeout",
        14: "Over Load",
    }
    out['status'] = ERR_MAP.get(out['err_code'], f"Unknown ({out['err_code']})")

    # MIT feedback payload uses unsigned packed values: P(16), V(12), T(12)
    pos_uint = (D[1] << 8) | D[2]
    vel_uint = (D[3] << 4) | (D[4] >> 4)
    tor_uint = ((D[4] & 0x0F) << 8) | D[5]

    out['pos_uint'] = pos_uint
    out['vel_uint'] = vel_uint
    out['tor_uint'] = tor_uint

    # Convert packed uint values back to physical values with MIT ranges
    out['pos'] = uint_to_float(pos_uint, pmin, pmax, 16)
    out['vel'] = uint_to_float(vel_uint, vmin, vmax, 12)
    out['tor'] = uint_to_float(tor_uint, tmin, tmax, 12)

    # MOS temp: D[6], Rotor temp: D[7]
    out['temp_mos'] = D[6]
    out['tempRotor'] = D[7]
    out['temp_rotor'] = D[7]

    return out

# ----------------------------
# CAN worker thread
# ----------------------------

class CanReaderThread(QThread):
    message_received = Signal(object)  # will emit can.Message or tuple (arb_id, data)

    def __init__(self, bus: can.BusABC):
        super().__init__()
        self.bus = bus
        self._running = True

    def run(self):
        # Use blocking recv with timeout to allow graceful stop
        while self._running:
            try:
                msg = self.bus.recv(timeout=1.0)
            except Exception as e:
                # emit an error-like object
                self.message_received.emit({'error': str(e)})
                break
            if msg is None:
                continue
            # Emit the message
            self.message_received.emit(msg)

    def stop(self):
        self._running = False
        self.wait(2000)


# ----------------------------
# Main Window
# ----------------------------

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DM-H3510 Motor Debug (Qt6 + python-can)")
        self.resize(900, 600)
        self.bus = None
        self.reader = None

        # CSV logging state
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None

        # Cached command frames for periodic sending (updated by Send buttons)
        self.cached_frames: dict[int, Optional[tuple[int, bytes]]] = {
            0: None,  # MIT
            1: None,  # Pos-Vel
            2: None,  # Velocity
            3: None,  # Force-Position
        }
        
        # Timer for periodic sending
        self.send_timer = QTimer(self)
        self.send_timer.setInterval(20)  # 20ms = 50Hz
        self.send_timer.timeout.connect(self.on_send_timer)

        self._build_ui()
        self._init_default_cached_frames()

    def _start_csv_logging(self):
        if self.csv_file is not None:
            return
        try:
            log_dir = os.path.join(os.path.dirname(__file__), "logs")
            os.makedirs(log_dir, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            self.csv_path = os.path.join(log_dir, f"feedback_{ts}.csv")
            self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "timestamp",
                "arb_id",
                "position",
                "velocity",
                "torque",
                "controller_id",
                "err_code",
                "status",
                "temp_mos",
                "temp_rotor",
            ])
            self.csv_file.flush()
            self.log(f"CSV logging started: {self.csv_path}")
        except Exception as e:
            self.csv_file = None
            self.csv_writer = None
            self.csv_path = None
            self.log(f"CSV logging start failed: {e}")

    def _stop_csv_logging(self):
        if self.csv_file is None:
            return
        try:
            self.csv_file.flush()
            self.csv_file.close()
            self.log("CSV logging stopped")
        except Exception as e:
            self.log(f"CSV logging stop failed: {e}")
        finally:
            self.csv_file = None
            self.csv_writer = None

    def _write_feedback_csv(self, arb_id: int, parsed: dict):
        if self.csv_writer is None or self.csv_file is None:
            return
        try:
            csv_file = self.csv_file
            self.csv_writer.writerow([
                time.strftime("%Y-%m-%d %H:%M:%S"),
                hex(arb_id),
                parsed.get('pos'),
                parsed.get('vel'),
                parsed.get('tor'),
                parsed.get('controller_id'),
                parsed.get('err_code'),
                parsed.get('status'),
                parsed.get('temp_mos'),
                parsed.get('temp_rotor'),
            ])
            csv_file.flush()
        except Exception as e:
            self.log(f"CSV write failed: {e}")

    def _init_default_cached_frames(self):
        try:
            esc_id = self._get_esc_id()
        except Exception:
            esc_id = 0x01

        # MIT default cache
        mit_data = pack_mit_frame(0.0, 0.0, 10.0, 0.1, 0.0,
                                  pmin=-12.5, pmax=12.5,
                                  vmin=-280.0, vmax=280.0,
                                  tmin=-1.0, tmax=1.0)
        self.cached_frames[0] = (esc_id, mit_data)

        # Pos-Vel default cache
        self.cached_frames[1] = (0x100 + esc_id, float_to_le_bytes(0.0) + float_to_le_bytes(0.0))

        # Velocity default cache
        self.cached_frames[2] = (0x200 + esc_id, float_to_le_bytes(0.0))

        # Force-Position default cache
        self.cached_frames[3] = (0x300 + esc_id, pack_force_position_frame(0.0, 10.0, 0.5))

    def _write_mode_register(self, mode_idx: int):
        if self.bus is None:
            return
        try:
            esc_id = self._get_esc_id()
            mode_val = int(mode_idx) + 1  # 0..3 -> 1..4
            canid_l = esc_id & 0xFF
            canid_h = (esc_id >> 8) & 0xFF
            mode_data = struct.pack('<I', mode_val)
            payload = pack_write_register(canid_l, canid_h, 0x0A, mode_data)
            self._send_frame(0x7FF, payload)
            self.log(f"Mode changed -> write RID 0x0A = {mode_val}")
        except Exception as e:
            self.log(f"Mode write failed: {e}")

    def on_mode_changed(self, idx: int):
        self._write_mode_register(idx)

    def _fill_default_if_empty(self, line_edit: QLineEdit, default_value: str) -> str:
        text = line_edit.text().strip()
        if text == "":
            line_edit.setText(default_value)
            return default_value
        return text

    def _float_from_edit(self, line_edit: QLineEdit, default_value: str, field_name: str) -> float:
        text = self._fill_default_if_empty(line_edit, default_value)
        try:
            return float(text)
        except Exception as e:
            raise ValueError(f"{field_name} invalid: {e}")

    def _int_from_edit(self, line_edit: QLineEdit, default_value: str, field_name: str) -> int:
        text = self._fill_default_if_empty(line_edit, default_value)
        try:
            return int(text, 0)
        except Exception as e:
            raise ValueError(f"{field_name} invalid: {e}")

    def _build_ui(self):
        layout = QVBoxLayout(self)

        # Connection controls
        conn_layout = QHBoxLayout()
        conn_layout.addWidget(QLabel("Serial/Port:"))
        default_port = "can0" if sys.platform.startswith("linux") else "COM13"
        self.port_edit = QLineEdit(default_port)
        conn_layout.addWidget(self.port_edit)
        conn_layout.addWidget(QLabel("Baud (slcan serial):"))
        self.baud_edit = QLineEdit("115200")
        conn_layout.addWidget(self.baud_edit)
        conn_layout.addWidget(QLabel("CAN bitrate (bps):"))
        self.canrate_combo = QComboBox()
        self.canrate_combo.addItems(["1000000","500000","250000","125000"])
        conn_layout.addWidget(self.canrate_combo)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connect)
        conn_layout.addWidget(self.btn_connect)
        layout.addLayout(conn_layout)

        # Real-time Feedback
        fb_layout = QHBoxLayout()
        fb_layout.addWidget(QLabel("Real-time Feedback:"))
        self.lbl_fb_pos = QLineEdit("Pos: 0.00"); self.lbl_fb_pos.setReadOnly(True)
        self.lbl_fb_vel = QLineEdit("Vel: 0.00"); self.lbl_fb_vel.setReadOnly(True)
        self.lbl_fb_tor = QLineEdit("Tor: 0.00"); self.lbl_fb_tor.setReadOnly(True)
        fb_layout.addWidget(self.lbl_fb_pos)
        fb_layout.addWidget(self.lbl_fb_vel)
        fb_layout.addWidget(self.lbl_fb_tor)
        layout.addLayout(fb_layout)

        # Mode Selection
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Control Mode:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["MIT", "Pos-Vel", "Velocity", "Force-Position"])
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        mode_layout.addWidget(self.mode_combo)
        mode_layout.addStretch()
        layout.addLayout(mode_layout)

        # Controls / IDs
        grid = QGridLayout()
        grid.addWidget(QLabel("ESC ID (hex)"), 0, 0)
        self.esc_id_edit = QLineEdit("0x01")
        grid.addWidget(self.esc_id_edit, 0, 1)

        grid.addWidget(QLabel("MST ID (hex)"), 0, 2)
        self.mst_id_edit = QLineEdit("0x00")
        grid.addWidget(self.mst_id_edit, 0, 3)

        grid.addWidget(QLabel("PMAX (rad)"), 1, 0)
        self.pmax_edit = QLineEdit("12.5")
        grid.addWidget(self.pmax_edit, 1, 1)
        grid.addWidget(QLabel("VMAX (rad/s)"), 1, 2)
        self.vmax_edit = QLineEdit("280.0")
        grid.addWidget(self.vmax_edit, 1, 3)

        grid.addWidget(QLabel("TMAX (A norm)"), 2, 0)
        self.tmax_edit = QLineEdit("1.0")
        grid.addWidget(self.tmax_edit, 2, 1)

        layout.addLayout(grid)

        # Action buttons
        actions = QHBoxLayout()
        self.btn_enable = QPushButton("Enable")
        self.btn_enable.clicked.connect(self.do_enable)
        actions.addWidget(self.btn_enable)
        self.btn_disable = QPushButton("Disable")
        self.btn_disable.clicked.connect(self.do_disable)
        actions.addWidget(self.btn_disable)
        self.btn_save_zero = QPushButton("Save Zero")
        self.btn_save_zero.clicked.connect(self.do_save_zero)
        actions.addWidget(self.btn_save_zero)
        self.btn_clear = QPushButton("Clear Error")
        self.btn_clear.clicked.connect(self.do_clear_error)
        actions.addWidget(self.btn_clear)
        self.btn_store = QPushButton("Store Params (flash)")
        self.btn_store.clicked.connect(self.do_store_params)
        actions.addWidget(self.btn_store)
        layout.addLayout(actions)

        # Control send area
        ctrl_layout = QHBoxLayout()
        # left: MIT inputs
        mit_box = QVBoxLayout()
        mit_box.addWidget(QLabel("MIT Mode (bitpacked)"))
        mit_row = QHBoxLayout()
        self.mit_p = QLineEdit("0.0"); mit_row.addWidget(QLabel("p_des")); mit_row.addWidget(self.mit_p)
        self.mit_v = QLineEdit("0.0"); mit_row.addWidget(QLabel("v_des")); mit_row.addWidget(self.mit_v)
        mit_box.addLayout(mit_row)
        mit_row2 = QHBoxLayout()
        self.mit_kp = QLineEdit("10.0"); mit_row2.addWidget(QLabel("Kp")); mit_row2.addWidget(self.mit_kp)
        self.mit_kd = QLineEdit("0.1"); mit_row2.addWidget(QLabel("Kd")); mit_row2.addWidget(self.mit_kd)
        mit_box.addLayout(mit_row2)
        mit_row3 = QHBoxLayout()
        self.mit_t = QLineEdit("0.0"); mit_row3.addWidget(QLabel("t_ff")); mit_row3.addWidget(self.mit_t)
        self.btn_send_mit = QPushButton("Send MIT")
        self.btn_send_mit.clicked.connect(self.send_mit)
        mit_row3.addWidget(self.btn_send_mit)
        mit_box.addLayout(mit_row3)
        ctrl_layout.addLayout(mit_box)

        # middle: position/velocity
        pv_box = QVBoxLayout()
        pv_box.addWidget(QLabel("Position/Velocity Mode"))
        pv_row = QHBoxLayout()
        self.pos_p = QLineEdit("0.0"); pv_row.addWidget(QLabel("p_des")); pv_row.addWidget(self.pos_p)
        self.pos_v = QLineEdit("0.0"); pv_row.addWidget(QLabel("v_des")); pv_row.addWidget(self.pos_v)
        pv_box.addLayout(pv_row)
        self.btn_send_pos = QPushButton("Send PositionVelocity")
        self.btn_send_pos.clicked.connect(self.send_posvel)
        pv_box.addWidget(self.btn_send_pos)

        vel_row = QHBoxLayout()
        self.vel_v = QLineEdit("0.0"); vel_row.addWidget(QLabel("v_des")); vel_row.addWidget(self.vel_v)
        self.btn_send_vel = QPushButton("Send Velocity")
        self.btn_send_vel.clicked.connect(self.send_velocity)
        vel_row.addWidget(self.btn_send_vel)
        pv_box.addLayout(vel_row)

        ctrl_layout.addLayout(pv_box)

        # Force-Position
        fp_box = QVBoxLayout()
        fp_box.addWidget(QLabel("Force-Position Mode"))
        fp_row = QHBoxLayout()
        self.fp_p = QLineEdit("0.0"); fp_row.addWidget(QLabel("p_des")); fp_row.addWidget(self.fp_p)
        self.fp_v = QLineEdit("10.0"); fp_row.addWidget(QLabel("v_des")); fp_row.addWidget(self.fp_v)
        fp_box.addLayout(fp_row)
        fp_row2 = QHBoxLayout()
        self.fp_i = QLineEdit("0.5"); fp_row2.addWidget(QLabel("i_des")); fp_row2.addWidget(self.fp_i)
        self.btn_send_fp = QPushButton("Send Force-Pos")
        self.btn_send_fp.clicked.connect(self.send_force_pos)
        fp_row2.addWidget(self.btn_send_fp)
        fp_box.addLayout(fp_row2)
        ctrl_layout.addLayout(fp_box)

        # right: reg read/write
        reg_box = QVBoxLayout()
        reg_box.addWidget(QLabel("Register read/write"))
        rrow = QHBoxLayout()
        self.reg_rid = QLineEdit("0x50")
        rrow.addWidget(QLabel("RID"))
        rrow.addWidget(self.reg_rid)
        self.btn_read_reg = QPushButton("Read")
        self.btn_read_reg.clicked.connect(self.read_reg)
        rrow.addWidget(self.btn_read_reg)
        reg_box.addLayout(rrow)

        # Write register row with type selection and two input fields
        wrow = QHBoxLayout()
        self.reg_wrid = QLineEdit("0x00")
        wrow.addWidget(QLabel("RID"))
        wrow.addWidget(self.reg_wrid)

        self.reg_data_type = QComboBox()
        self.reg_data_type.addItems(["Float", "Uint32"])
        self.reg_data_type.currentTextChanged.connect(self._on_reg_data_type_changed)
        wrow.addWidget(self.reg_data_type)

        self.reg_wdata_label = QLabel("Data:")
        wrow.addWidget(self.reg_wdata_label)
        self.reg_wdata_edit = QLineEdit("0.0")
        wrow.addWidget(self.reg_wdata_edit)

        self.btn_write_reg = QPushButton("Write")
        self.btn_write_reg.clicked.connect(self.write_reg)
        wrow.addWidget(self.btn_write_reg)
        reg_box.addLayout(wrow)

        self._on_reg_data_type_changed(self.reg_data_type.currentText())

        ctrl_layout.addLayout(reg_box)

        layout.addLayout(ctrl_layout)

        # Log and parsed output
        out_layout = QHBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        out_layout.addWidget(self.log_text, 2)
        self.parsed_text = QTextEdit()
        self.parsed_text.setReadOnly(True)
        out_layout.addWidget(self.parsed_text, 1)
        layout.addLayout(out_layout)

    def _on_reg_data_type_changed(self, data_type: str):
        if data_type == "Float":
            self.reg_wdata_label.setText("Float:")
            self.reg_wdata_edit.setPlaceholderText("e.g. 0.0")
            if self.reg_wdata_edit.text().strip() == "":
                self.reg_wdata_edit.setText("0.0")
        else:
            self.reg_wdata_label.setText("Uint32:")
            self.reg_wdata_edit.setPlaceholderText("e.g. 0 or 0x1234")
            if self.reg_wdata_edit.text().strip() == "":
                self.reg_wdata_edit.setText("0")

    def log(self, s: str):
        ts = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {s}")

    def toggle_connect(self):
        if self.bus is None:
            # connect
            default_port = "can0" if sys.platform.startswith("linux") else "COM13"
            port = self._fill_default_if_empty(self.port_edit, default_port)
            try:
                baud = self._int_from_edit(self.baud_edit, "115200", "Serial baud")
            except ValueError as e:
                QMessageBox.warning(self, "Input error", str(e))
                return
            canrate = int(self.canrate_combo.currentText())
            try:
                if port.startswith("can"):
                    # Use socketcan for CAN interfaces in Linux
                    self.log(f"Opening socketcan on {port}, CAN {canrate} bps...")
                    self.bus = can.interface.Bus(interface='socketcan', channel=port, bitrate=canrate)
                else:
                    # create slcan Bus via python-can
                    # bustype 'slcan' should use pyserial under the hood
                    self.log(f"Opening slcan on {port} serial {baud} bps, CAN {canrate} bps...")
                    self.bus = can.interface.Bus(interface='slcan', channel=port, bitrate=canrate, serial_baudrate=baud)
            except Exception as e:
                QMessageBox.critical(self, "Connect failed", f"Failed to open interface: {e}")
                self.bus = None
                return
            # start reader thread
            self.reader = CanReaderThread(self.bus)
            self.reader.message_received.connect(self.on_can_message)
            self.reader.start()
            self._start_csv_logging()
            self._write_mode_register(self.mode_combo.currentIndex())
            self.btn_connect.setText("Disconnect")
            self.log("Connected")
        else:
            # disconnect
            self.send_timer.stop()
            self._stop_csv_logging()
            if self.reader:
                self.reader.stop()
                self.reader = None
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
            self.btn_connect.setText("Connect")
            self.log("Disconnected")

    @Slot(object)
    def on_can_message(self, msg):
        # msg may be can.Message or dict-like error/status object
        if isinstance(msg, dict):
            if 'error' in msg:
                self.log(f"CAN read error: {msg['error']}")
            else:
                self.log(f"CAN read info: {msg}")
            return

        if not isinstance(msg, can.Message):
            self.log(f"Unknown CAN message type: {type(msg).__name__}")
            return

        try:
            arb = hex(msg.arbitration_id)
            data = bytes(msg.data)
            self.log(f"Recv ID={arb} len={len(data)} data={data.hex()}")

            # Try to get MST_ID from GUI
            try:
                mst_id = int(self.mst_id_edit.text(), 0)
            except Exception:
                mst_id = None

            # Try to get ESC_ID from GUI
            try:
                esc_id = int(self.esc_id_edit.text(), 0)
            except Exception:
                esc_id = None

            # Check if this is a register response (read/write/store)
            if mst_id is not None and msg.arbitration_id == mst_id and (data[1] << 8 | data[0]) == esc_id and len(data) >= 4:
                # Check for read response (0x33)
                if data[2] == 0x33:
                    rid = data[3]
                    value_bytes = data[4:8] if len(data) >= 8 else b''
                    value_int = int.from_bytes(value_bytes, 'little') if len(value_bytes) == 4 else None
                    value_float = struct.unpack('<f', value_bytes)[0] if len(value_bytes) == 4 else None
                    self.parsed_text.append(f"Read response: RID=0x{rid:02X}, raw={value_bytes.hex()}, as uint32={value_int}, as float={value_float}")
                    return
                elif data[2] == 0x55:
                    rid = data[3]
                    value_bytes = data[4:8] if len(data) >= 8 else b''
                    value_int = int.from_bytes(value_bytes, 'little') if len(value_bytes) == 4 else None
                    value_float = struct.unpack('<f', value_bytes)[0] if len(value_bytes) == 4 else None
                    self.parsed_text.append(f"Write response: RID=0x{rid:02X}, written data: raw={value_bytes.hex()}, uint32={value_int}, float={value_float}")
                    return
                elif data[2] == 0xAA and data[3] == 0x01:
                    self.parsed_text.append("Store parameters response (success)")
                    return

            # Otherwise, attempt to parse as feedback frame
            if isinstance(msg.arbitration_id, int) and len(data) == 8:
                try:
                    pmax = float(self.pmax_edit.text().strip() or "12.5")
                except Exception:
                    pmax = 12.5
                try:
                    vmax = float(self.vmax_edit.text().strip() or "45.0")
                except Exception:
                    vmax = 45.0
                try:
                    tmax = float(self.tmax_edit.text().strip() or "18.0")
                except Exception:
                    tmax = 18.0
                parsed = parse_feedback_frame(
                    data,
                    pmin=-abs(pmax), pmax=abs(pmax),
                    vmin=-abs(vmax), vmax=abs(vmax),
                    tmin=-abs(tmax), tmax=abs(tmax),
                )
                
                if 'pos' in parsed:
                    self.lbl_fb_pos.setText(f"Pos: {parsed['pos']:.2f}")
                if 'vel' in parsed:
                    self.lbl_fb_vel.setText(f"Vel: {parsed['vel']:.2f}")
                if 'tor' in parsed:
                    self.lbl_fb_tor.setText(f"Tor: {parsed['tor']:.2f}")

                self._write_feedback_csv(msg.arbitration_id, parsed)
                
                self.parsed_text.append(f"ID={hex(msg.arbitration_id)} parsed: {parsed}")
            else:
                self.parsed_text.append(f"ID={hex(msg.arbitration_id)} raw: {data.hex()}")
        except Exception as e:
            self.log(f"Error in on_can_message: {e}")

    def _get_esc_id(self):
        return self._int_from_edit(self.esc_id_edit, "0x01", "ESC ID")

    def _send_frame(self, arb_id: int, data: bytes, ext=False):
        if self.bus is None:
            QMessageBox.warning(self, "Not connected", "Please connect to the slcan device first.")
            return
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=ext)
        try:
            self.bus.send(msg, timeout=1.0)
            self.log(f"Sent ID={hex(arb_id)} data={data.hex()}")
        except Exception as e:
            self.log(f"Send failed: {e}")

    # Control commands
    def do_enable(self):
        # Enable motor, periodic sending uses cached frame only
        try:
            ESC = self._get_esc_id()
        except Exception:
            ESC = 1
        self._send_frame(ESC, pack_enable())
        self.send_timer.start(20)

    def do_disable(self):
        self.send_timer.stop()
        eid = self._get_esc_id()
        self._send_frame(eid, pack_disable())

    def on_send_timer(self):
        if self.bus is None:
            return
        idx = self.mode_combo.currentIndex()  # 0=MIT, 1=PosVel, 2=Vel, 3=ForcePos
        cached = self.cached_frames.get(idx)
        if cached is None:
            return
        arb_id, data = cached
        self._send_frame(arb_id, data)

    def do_save_zero(self):
        eid = self._get_esc_id()
        self._send_frame(eid, pack_save_zero())

    def do_clear_error(self):
        eid = self._get_esc_id()
        self._send_frame(eid, pack_clear_error())

    def do_store_params(self):
        # By spec: send to ID=0x7FF: D0..D3 = MST_ID STD CANID_L CANID_H 0xAA 0x01? We'll follow doc: send with ID 0x7FF and payload [STD_CANID_L, STD_CANID_H, 0xAA, 0x01]
        # STD CAN ID is the master ID low and high? We'll use MST ID low/high as zeros for common case.
        try:
            ESC = self._int_from_edit(self.esc_id_edit, "0x01", "ESC ID")
        except ValueError:
            ESC = 0
        canid_l = ESC & 0xFF
        canid_h = (ESC >> 8) & 0xFF
        payload = bytes([canid_l, canid_h, 0xAA, 0x01])
        self._send_frame(0x7FF, payload)

    # Mode frames
    def send_mit(self):
        try:
            p = self._float_from_edit(self.mit_p, "0.0", "MIT p_des")
            v = self._float_from_edit(self.mit_v, "0.0", "MIT v_des")
            kp = self._float_from_edit(self.mit_kp, "10.0", "MIT Kp")
            kd = self._float_from_edit(self.mit_kd, "0.1", "MIT Kd")
            t = self._float_from_edit(self.mit_t, "0.0", "MIT t_ff")
            pmax = self._float_from_edit(self.pmax_edit, "3.14159", "PMAX")
            vmax = self._float_from_edit(self.vmax_edit, "100.0", "VMAX")
            tmax = self._float_from_edit(self.tmax_edit, "1.0", "TMAX")
        except Exception as e:
            QMessageBox.warning(self, "Input error", f"Invalid MIT parameters: {e}")
            return
        # Use symmetric ranges based on user-set maxima
        data = pack_mit_frame(p, v, kp, kd, t,
                              pmin=-pmax, pmax=pmax,
                              vmin=-vmax, vmax=vmax,
                              tmin=-tmax, tmax=tmax)
        eid = self._get_esc_id()
        self.cached_frames[0] = (eid, data)
        self._send_frame(eid, data)

    def send_posvel(self):
        # PositionVelocity uses ID = 0x100 + ID, payload: p_des (float32, low first), v_des (float32)
        try:
            p = self._float_from_edit(self.pos_p, "0.0", "Position p_des")
            v = self._float_from_edit(self.pos_v, "0.0", "Position v_des")
        except Exception as e:
            QMessageBox.warning(self, "Input error", f"Invalid p/v: {e}")
            return
        payload = float_to_le_bytes(p) + float_to_le_bytes(v)
        eid = 0x100 + self._get_esc_id()
        self.cached_frames[1] = (eid, payload)
        self._send_frame(eid, payload)

    def send_velocity(self):
        # Velocity uses ID = 0x200 + ID, payload: v_des (float32, low first) — doc shows 4 bytes
        try:
            v = self._float_from_edit(self.vel_v, "0.0", "Velocity v_des")
        except Exception as e:
            QMessageBox.warning(self, "Input error", f"Invalid v: {e}")
            return
        payload = float_to_le_bytes(v)
        eid = 0x200 + self._get_esc_id()
        self.cached_frames[2] = (eid, payload)
        self._send_frame(eid, payload)

    def send_force_pos(self):
        try:
            p = self._float_from_edit(self.fp_p, "0.0", "FP p_des")
            v = self._float_from_edit(self.fp_v, "0.0", "FP v_des")
            i = self._float_from_edit(self.fp_i, "0.0", "FP i_des")
        except Exception as e:
            # Avoid popup in timer loop, just log
            self.log(f"FP input error: {e}")
            return
        
        data = pack_force_position_frame(p, v, i)
        # ID is 0x300 + ESC_ID
        eid = 0x300 + self._get_esc_id()
        self.cached_frames[3] = (eid, data)
        self._send_frame(eid, data)

    # register read/write
    def read_reg(self):
        try:
            rid = self._int_from_edit(self.reg_rid, "0x50", "Read RID")
            ESC = self._int_from_edit(self.esc_id_edit, "0x01", "ESC ID")
        except Exception as e:
            QMessageBox.warning(self, "Input error", f"Invalid RID/ESC: {e}")
            return
        canid_l = ESC & 0xFF
        canid_h = (ESC >> 8) & 0xFF
        payload = pack_read_register(canid_l, canid_h, rid)
        # per doc, send with arbitration_id = 0x7FF
        self._send_frame(0x7FF, payload)

    def write_reg(self):
        try:
            rid = self._int_from_edit(self.reg_wrid, "0x00", "Write RID")
            data_type = self.reg_data_type.currentText()
            if data_type == "Float":
                val = self._float_from_edit(self.reg_wdata_edit, "0.0", "Write Data (float)")
                data4 = float_to_le_bytes(val)
            else:  # Uint32
                val = self._int_from_edit(self.reg_wdata_edit, "0", "Write Data (uint32)")
                data4 = struct.pack('<I', val)  # 4-byte little-endian unsigned int
            ESC = self._int_from_edit(self.esc_id_edit, "0x01", "ESC ID")
        except Exception as e:
            QMessageBox.warning(self, "Input error", f"Invalid inputs: {e}")
            return
        canid_l = ESC & 0xFF
        canid_h = (ESC >> 8) & 0xFF
        payload = pack_write_register(canid_l, canid_h, rid, data4)
        self._send_frame(0x7FF, payload)


# ----------------------------
# Run app
# ----------------------------

def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()