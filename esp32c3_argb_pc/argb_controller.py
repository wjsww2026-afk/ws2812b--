#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32-C3 神光同步 —— PC 端控制工作台（PySide6）

通过串口控制多路 WS2812B 灯带（最多 4 路，引脚 / 灯珠数可运行时配置，无需预先知道
实际灯珠数量），支持：
灯效 / 多色渐变 / 参数化 / 分段 / 像素画 / 编排轮播 / 音乐律动 / 预设
各路可拼接成一条长灯带，也可独立同时跑同一灯效。

依赖：pip install PySide6 pyserial              （必需）
      pip install soundcard numpy              （音乐律动，可选；可采集系统声音）
      pip install sounddevice                  （可选；仅麦克风时作为备用通道）
运行：python argb_controller.py
"""

import json
import os
import queue
import sys
import threading

from PySide6.QtCore import (Qt, QObject, QThread, QTimer, QSettings, Signal, Slot,
                            QRectF, QEasingCurve, QPropertyAnimation, QAbstractAnimation)
from PySide6.QtGui import (QColor, QPainter, QPen, QLinearGradient, QIcon,
                           QPixmap, QAction)
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QFrame, QLabel,
                               QPushButton, QCheckBox, QComboBox, QSpinBox, QSlider,
                               QListWidget, QGridLayout, QHBoxLayout, QVBoxLayout,
                               QStackedWidget, QPlainTextEdit, QButtonGroup,
                               QGraphicsOpacityEffect, QSizePolicy, QColorDialog,
                               QFileDialog, QMessageBox, QScrollArea,
                               QSystemTrayIcon, QMenu)

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少依赖，请先安装：pip install PySide6 pyserial")
    sys.exit(1)

try:
    import numpy as np
    HAVE_NP = True
except Exception:
    HAVE_NP = False

try:
    import soundcard as sc          # 系统声音（扬声器回环）+ 麦克风
    HAVE_SC = True
except Exception:
    HAVE_SC = False

try:
    import sounddevice as sd        # 备用通道：仅麦克风
    HAVE_SD = True
except Exception:
    HAVE_SD = False

AUDIO_OK = HAVE_NP and (HAVE_SC or HAVE_SD)

BAUD = 115200
LED_COUNT = 14                 # 默认灯珠数（单路默认）
MAX_SEG = 4
MAX_STEP = 8
MAX_STRIPS = 4                 # 最多支持的灯带路数
MAX_PER_STRIP = 128            # 单路灯珠上限
MAX_TOTAL_PIXELS = 512         # 逻辑灯珠总量上限
GPIO_CHOICES = [0, 1, 2, 3, 4, 5, 6, 7, 10]   # ESP32-C3 可用 GPIO
# 路径处理：打包(PyInstaller)后 HERE 指向 exe 同级目录，用户数据（presets.json）写这里；
# RES_DIR 指向随包资源目录（onefile 会解包到 _MEIPASS），只读资源（icon.ico）从这里取
FROZEN = getattr(sys, "frozen", False)
HERE = (os.path.dirname(os.path.abspath(sys.executable)) if FROZEN
        else os.path.dirname(os.path.abspath(__file__)))
RES_DIR = getattr(sys, "_MEIPASS", HERE)
PRESET_FILE = os.path.join(HERE, "presets.json")
ICON_FILE = os.path.join(RES_DIR, "icon.ico")

MODES = [("关闭", 0), ("静态", 1), ("呼吸", 2), ("彩虹", 3), ("流星", 4), ("流光", 5),
         ("渐变", 6), ("正弦波", 7), ("心跳", 8), ("警灯", 9), ("星空", 10),
         ("色块", 11), ("双色交替", 12), ("像素画", 13), ("音乐律动", 14),
         ("立体声律动", 15)]
MODE_NAMES = [m[0] for m in MODES]
MODE_BY_NAME = {m[0]: m[1] for m in MODES}
MODE_NAME_BY_VAL = {m[1]: m[0] for m in MODES}

PALETTE = ["#FF0000", "#FF8000", "#FFFF00", "#00FF00", "#00FFFF",
           "#0000FF", "#8000FF", "#FF00FF", "#FFFFFF", "#000000"]

QSS = """
* { font-family: "Microsoft YaHei UI", "Microsoft YaHei", "Segoe UI"; }

QWidget#Root { background: #ffffff; }

QFrame#Sidebar { background: #ffffff; border-right: 1px solid #eceef6; }
QLabel#AppTitle { font-size: 17px; font-weight: 600; color: #1f2333; }
QLabel#AppSub   { font-size: 11px; color: #9aa0b6; }
QLabel#Status   { font-size: 11px; color: #9aa0b6; }

QPushButton#NavBtn {
    text-align: left; padding: 0 14px; border: none; border-radius: 9px;
    background: transparent; color: #5b6178; font-size: 13px;
}
QPushButton#NavBtn:hover { background: #f2f3fa; color: #2c3047; }
QPushButton#NavBtn:checked {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #667eea, stop:1 #8a74e8);
    color: #ffffff; font-weight: 600;
}

QFrame#Card { background: #ffffff; border: 1px solid #e9eaf2; border-radius: 12px; }
QLabel#CardTitle { font-size: 12px; font-weight: 600; color: #6c7290; }
QLabel#Field { font-size: 12px; color: #4a5068; }
QLabel#Value { font-size: 12px; color: #1f2333; }
QLabel#Mono  { font-size: 12px; color: #4a5068; font-family: Consolas, monospace; }
QLabel#Hint  { font-size: 11px; color: #9aa0b6; }

QPushButton {
    background: #ffffff; border: 1px solid #dcdef0; border-radius: 8px;
    padding: 6px 12px; color: #3b4056; font-size: 12px;
}
QPushButton:hover { background: #f5f6fd; border-color: #c3c7ea; }
QPushButton:pressed { background: #eaecfb; }
QPushButton:disabled { color: #b3b7c9; background: #f7f7fb; }

QPushButton#Accent {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #667eea, stop:1 #8a74e8);
    border: 1px solid transparent; color: #ffffff; font-weight: 600;
}
QPushButton#Accent:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #7a8ff0, stop:1 #9b87ee);
}
QPushButton#Accent:pressed {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #5668d6, stop:1 #7a63d8);
}

QPushButton#Mode {
    background: #ffffff; border: 1px solid #e3e4f2; border-radius: 9px;
    padding: 7px 2px; color: #4a5068; font-size: 12px;
}
QPushButton#Mode:hover { background: #f6f7fd; border-color: #bcc0e6; }
QPushButton#Mode:checked {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #667eea, stop:1 #8a74e8);
    border: 1px solid #667eea; color: #ffffff; font-weight: 600;
}

QCheckBox { spacing: 7px; color: #3b4056; font-size: 12px; }
QCheckBox::indicator { width: 16px; height: 16px; border: 1px solid #cfd2e8;
                       border-radius: 5px; background: #ffffff; }
QCheckBox::indicator:hover { border-color: #9aa0d0; }
QCheckBox::indicator:checked {
    border: 1px solid #667eea;
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #667eea, stop:1 #8a74e8);
}

QSlider::groove:horizontal { height: 6px; background: #edeef7; border-radius: 3px; }
QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #667eea, stop:1 #9a86ee);
    border-radius: 3px;
}
QSlider::handle:horizontal { width: 14px; height: 14px; margin: -5px 0;
                             border-radius: 8px; background: #ffffff; border: 2px solid #667eea; }
QSlider::handle:horizontal:hover { border-color: #4658e0; }

QComboBox {
    background: #ffffff; border: 1px solid #dcdef0; border-radius: 8px;
    padding: 5px 22px 5px 10px; color: #3b4056; font-size: 12px; min-height: 18px;
}
QComboBox:hover { border-color: #c3c7ea; }
QComboBox:focus { border-color: #667eea; }
QComboBox::drop-down { border: none; width: 22px; }
QComboBox::down-arrow { width: 0; height: 0; margin-right: 8px;
    border-left: 4px solid transparent; border-right: 4px solid transparent;
    border-top: 5px solid #8a90a6; }
QComboBox QAbstractItemView {
    background: #ffffff; border: 1px solid #e3e4f2; border-radius: 8px;
    padding: 4px; outline: none;
    selection-background-color: #eef0fe; selection-color: #4a3fbf;
}

QSpinBox {
    background: #ffffff; border: 1px solid #dcdef0; border-radius: 8px;
    padding: 4px 6px; color: #3b4056; font-size: 12px;
}
QSpinBox:focus { border-color: #667eea; }
QSpinBox::up-button, QSpinBox::down-button { width: 14px; border: none; background: transparent; }
QSpinBox::up-arrow { width: 0; height: 0;
    border-left: 3px solid transparent; border-right: 3px solid transparent;
    border-bottom: 4px solid #8a90a6; }
QSpinBox::down-arrow { width: 0; height: 0;
    border-left: 3px solid transparent; border-right: 3px solid transparent;
    border-top: 4px solid #8a90a6; }

QListWidget {
    background: #fbfbfe; border: 1px solid #e9eaf2; border-radius: 10px;
    padding: 5px; outline: none; font-size: 12px;
}
QListWidget::item { padding: 7px 9px; border-radius: 7px; color: #3b4056; }
QListWidget::item:hover { background: #f1f2f9; }
QListWidget::item:selected { background: #eef0fe; color: #4a3fbf; font-weight: 600; }

QPlainTextEdit#Log {
    background: #fbfbfe; border: 1px solid #e9eaf2; border-radius: 10px;
    padding: 7px; color: #4a5068;
    font-family: Consolas, "Cascadia Mono", monospace; font-size: 11px;
}

QScrollBar:vertical { background: transparent; width: 9px; margin: 2px; }
QScrollBar::handle:vertical { background: #d7d9e9; border-radius: 4px; min-height: 28px; }
QScrollBar::handle:vertical:hover { background: #b9bcd6; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }

QToolTip { background: #2c3047; color: #ffffff; border: none; padding: 5px 8px;
           border-radius: 6px; font-size: 11px; }
"""


def hex2rgb(h):
    return (int(h[1:3], 16), int(h[3:5], 16), int(h[5:7], 16))


def rgb2hex(c):
    return "#%02X%02X%02X" % (c[0], c[1], c[2])


def make_card(title=None):
    card = QFrame()
    card.setObjectName("Card")
    lay = QVBoxLayout(card)
    lay.setContentsMargins(14, 12, 14, 14)
    lay.setSpacing(10)
    if title:
        lbl = QLabel(title)
        lbl.setObjectName("CardTitle")
        lay.addWidget(lbl)
    return card, lay


# ---------------------------------------------------------------- 自绘控件
class Swatch(QWidget):
    """圆角色块，用于展示当前颜色。"""

    def __init__(self, color=(0, 0, 0), size=(46, 30), parent=None):
        super().__init__(parent)
        self._c = QColor(color[0], color[1], color[2])
        self.setFixedSize(size[0], size[1])

    def set_color(self, c):
        self._c = QColor(c[0], c[1], c[2])
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = QRectF(0.5, 0.5, self.width() - 1, self.height() - 1)
        p.setPen(QPen(QColor(214, 216, 232)))
        p.setBrush(self._c)
        p.drawRoundedRect(r, 7, 7)


class GradientPreview(QWidget):
    """14 颗灯珠的起色→终色渐变预览。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.c1 = QColor(0, 150, 255)
        self.c2 = QColor(0, 0, 0)
        self.setFixedHeight(48)

    def set_colors(self, c1, c2):
        self.c1 = QColor(c1[0], c1[1], c1[2])
        self.c2 = QColor(c2[0], c2[1], c2[2])
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        gap = 4.0
        cw = (self.width() - gap * (LED_COUNT - 1)) / float(LED_COUNT)
        for i in range(LED_COUNT):
            t = i / float(LED_COUNT - 1) if LED_COUNT > 1 else 0.0
            col = QColor(
                int(self.c1.red() + (self.c2.red() - self.c1.red()) * t),
                int(self.c1.green() + (self.c2.green() - self.c1.green()) * t),
                int(self.c1.blue() + (self.c2.blue() - self.c1.blue()) * t))
            p.setPen(QPen(QColor(224, 226, 240)))
            p.setBrush(col)
            p.drawRoundedRect(QRectF(i * (cw + gap), 0, cw, self.height()), 7, 7)


class PixelCanvas(QWidget):
    """按当前逻辑长度逐颗上色的画布（颗数未知也可用，横向可滚动）。"""

    cellClicked = Signal(int)

    MIN_CELL = 20

    def __init__(self, parent=None, count=LED_COUNT):
        super().__init__(parent)
        self.count = max(1, count)
        self.pix = [(0, 0, 0)] * self.count
        self.setFixedHeight(64)
        self.setMinimumWidth(self.count * self.MIN_CELL)
        self.setCursor(Qt.PointingHandCursor)

    def set_count(self, n):
        n = max(1, n)
        if n == self.count and len(self.pix) == n:
            return
        self.count = n
        if len(self.pix) < n:
            self.pix = self.pix + [(0, 0, 0)] * (n - len(self.pix))
        self.pix = self.pix[:n]
        self.setMinimumWidth(self.count * self.MIN_CELL)
        self.update()

    def set_pixels(self, pix):
        self.pix = list(pix)
        self.count = max(1, len(self.pix))
        self.setMinimumWidth(self.count * self.MIN_CELL)
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        h = self.height()
        cw = self.width() / float(self.count)
        for i in range(self.count):
            r = QRectF(i * cw + 3, 3, cw - 6, h - 6)
            c = self.pix[i]
            p.setPen(QPen(QColor(214, 216, 232)))
            p.setBrush(QColor(c[0], c[1], c[2]))
            p.drawRoundedRect(r, 7, 7)
            if cw >= 16:
                p.setPen(QColor(60, 64, 84) if sum(c) > 380 else QColor(190, 193, 210))
                p.drawText(r, Qt.AlignCenter, str(i))

    def mousePressEvent(self, e):
        i = int(e.position().x() / (self.width() / float(self.count)))
        if 0 <= i < self.count:
            self.cellClicked.emit(i)


class LevelMeter(QWidget):
    """音乐律动实时电平条。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.level = 0
        self.setFixedHeight(20)
        self.setMinimumWidth(120)

    def set_level(self, v):
        self.level = max(0, min(255, int(v)))
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()
        r = h / 2.0
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(240, 241, 248))
        p.drawRoundedRect(QRectF(0, 0, w, h), r, r)
        fw = w * self.level / 255.0
        if fw > 1:
            g = QLinearGradient(0, 0, w, 0)
            g.setColorAt(0.0, QColor(102, 126, 234))
            g.setColorAt(1.0, QColor(154, 134, 238))
            p.setBrush(g)
            p.drawRoundedRect(QRectF(0, 0, max(fw, h), h), r, r)


class StereoMeter(QWidget):
    """左右声道电平条：从中间向两端伸展，直观对应灯的律动方向。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.l = 0
        self.r = 0
        self.setFixedHeight(20)
        self.setMinimumWidth(120)

    def set_levels(self, l, r):
        self.l = max(0, min(255, int(l)))
        self.r = max(0, min(255, int(r)))
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()
        mid = w / 2.0
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(240, 241, 248))
        p.drawRoundedRect(QRectF(0, 0, w, h), h / 2.0, h / 2.0)
        for level, left in ((self.l, True), (self.r, False)):
            bw = mid * level / 255.0
            if bw <= 0.5:
                continue
            if left:
                rect = QRectF(mid - bw, 0, bw, h)
            else:
                rect = QRectF(mid, 0, bw, h)
            g = QLinearGradient(0, 0, w, 0)
            g.setColorAt(0.0, QColor(102, 126, 234))
            g.setColorAt(1.0, QColor(154, 134, 238))
            p.setBrush(g)
            p.drawRoundedRect(rect, 3, 3)
        p.setBrush(QColor(200, 203, 224))
        p.drawRect(QRectF(mid - 0.5, 0, 1, h))


# ---------------------------------------------------------------- 串口线程
class SerialWorker(QObject):
    """在独立线程里跑阻塞式串口收发，避免卡住界面。"""

    lineReceived = Signal(str)
    opened = Signal()
    closed = Signal()
    failed = Signal(str)

    def __init__(self):
        super().__init__()
        self._ser = None
        self._running = False
        self._port = None
        self._baud = BAUD
        self._tx = queue.Queue()

    def configure(self, port, baud):
        self._port = port
        self._baud = baud

    def enqueue(self, cmd):
        self._tx.put(cmd)

    def stop(self):
        self._running = False

    @Slot()
    def run(self):
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.05, write_timeout=0.5)
        except Exception as e:
            self.failed.emit("连接失败: %s" % e)
            return

        self._ser = ser
        self._running = True
        self.opened.emit()

        buf = b""
        try:
            while self._running:
                while not self._tx.empty():
                    cmd = self._tx.get_nowait()
                    try:
                        ser.write((cmd + "\n").encode("ascii", "ignore"))
                    except Exception as e:
                        self.failed.emit("发送失败: %s" % e)
                        self._running = False
                        break
                if not self._running:
                    break
                try:
                    n = ser.in_waiting
                    chunk = ser.read(n if n else 1)
                except Exception as e:
                    self.failed.emit("读取失败: %s" % e)
                    break
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        text = raw.decode("utf-8", "ignore").strip()
                        if text:
                            self.lineReceived.emit(text)
        finally:
            try:
                ser.close()
            except Exception:
                pass
            self._ser = None
            self._running = False
            self.closed.emit()


# ---------------------------------------------------------------- 主窗口
class MainWindow(QMainWindow):
    capError = Signal(str)              # 采集线程 -> 主线程日志

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32-C3 神光同步工作台")
        self.resize(1000, 780)
        self.setMinimumSize(900, 720)

        # 托盘与偏好设置（默认端口记忆在注册表，无需额外文件）
        self.settings = QSettings("ESP32C3-ARGB", "Controller")
        self.tray = None
        self.tray_ok = False
        self.act_conn = None
        self._really_quit = False        # 只有托盘「退出」才置真
        self._tray_tip_shown = False

        self.thread = None
        self.worker = None
        self.connected = False
        self._conn_port = ""
        self._synced = False            # 本次连接是否已成功同步到板子状态
        self.syncing = False            # 同步板子状态时抑制回调下发
        self._pending = {}              # 正在接收的 STATE...END 块
        self._deb = {}                  # 滑块去抖定时器

        self.state = {
            "mode": 2, "bright": 120, "speed": 50, "sync": 1,
            "r1": 0, "g1": 150, "b1": 255,
            "r2": 0, "g2": 0, "b2": 0,
            "hueSpan": 255, "dir": 0, "tail": 6, "spacing": 1,
            "mirror": 0, "random": 20, "gamma": 0, "gain": 0,
            "segCount": 0, "stepCount": 0, "playing": 0, "idle": 2,
        }
        self.segs = []                            # {start,length,mode,r,g,b}
        self.steps = []                           # {mode,secs,r1..b2}
        # 灯带硬件配置（引脚 / 颗数 / 拼接），与固件 STRIPS 状态块保持同步
        self.strip_cfg = {
            "numStrips": 1,
            "merge": 1,
            "strips": [{"pin": 10, "count": LED_COUNT, "enabled": 1}
                       for _ in range(MAX_STRIPS)],
        }
        self.logical_len = LED_COUNT               # 逻辑灯珠总数
        self.pix = [(0, 0, 0)] * LED_COUNT
        self.brush = (255, 0, 0)
        self.presets = {}
        self.seg_color = (255, 0, 0)
        self.step_c1 = (0, 150, 255)
        self.step_c2 = (255, 0, 120)

        self.music_run = False
        self.mic_level = 0
        self.level_l = 0
        self.level_r = 0
        self._cap_thread = None
        self._cap_stop = threading.Event()
        self._cap_kind = "mic"
        self._cap_dev = None
        self._music_timer = QTimer(self)
        self._music_timer.setInterval(30)
        self._music_timer.timeout.connect(self._music_tick)
        self._fade = None

        self.capError.connect(self.log)
        self._build()
        self.refresh_ports()
        self.refresh_devices()
        self.load_presets()
        self._build_tray()
        # 启动即自动连接默认端口（延迟到事件循环起来后执行）
        QTimer.singleShot(0, self._startup_autoconnect)

    # ------------------------------------------------------------ 界面骨架
    def _build(self):
        root = QWidget()
        root.setObjectName("Root")
        self.setCentralWidget(root)
        h = QHBoxLayout(root)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(0)

        h.addWidget(self._build_sidebar())

        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(16, 16, 16, 16)
        rv.setSpacing(12)
        rv.addWidget(self._build_topbar())

        self.stack = QStackedWidget()
        rv.addWidget(self.stack, 1)

        self._build_pages()
        self._build_log(rv)
        h.addWidget(right, 1)

    def _build_sidebar(self):
        bar = QFrame()
        bar.setObjectName("Sidebar")
        bar.setFixedWidth(172)
        v = QVBoxLayout(bar)
        v.setContentsMargins(12, 18, 12, 14)
        v.setSpacing(6)

        title = QLabel("神光同步")
        title.setObjectName("AppTitle")
        self.lbl_sub = QLabel("ESP32-C3 · WS2812B ×14")
        self.lbl_sub.setObjectName("AppSub")
        v.addWidget(title)
        v.addWidget(self.lbl_sub)
        v.addSpacing(16)

        self.nav_btns = []
        group = QButtonGroup(bar)
        group.setExclusive(True)
        for i, name in enumerate(["灯效", "颜色", "分段", "像素画", "编排", "音乐", "灯带"]):
            b = QPushButton(name)
            b.setObjectName("NavBtn")
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.setFixedHeight(36)
            group.addButton(b, i)
            v.addWidget(b)
            self.nav_btns.append(b)
        self.nav_btns[0].setChecked(True)
        group.idClicked.connect(self._switch_page)

        v.addStretch(1)

        self.chk_log = QCheckBox("显示日志")
        self.chk_log.setChecked(True)
        self.chk_log.toggled.connect(self._toggle_log)
        v.addWidget(self.chk_log)

        st = QWidget()
        sh = QHBoxLayout(st)
        sh.setContentsMargins(2, 8, 2, 0)
        sh.setSpacing(8)
        self.dot = QLabel()
        self.dot.setFixedSize(10, 10)
        self.status_lbl = QLabel("未连接")
        self.status_lbl.setObjectName("Status")
        sh.addWidget(self.dot)
        sh.addWidget(self.status_lbl)
        sh.addStretch(1)
        v.addWidget(st)
        self._set_status(False, "未连接")
        return bar

    def _set_status(self, ok, text):
        self.dot.setStyleSheet("background: %s; border-radius: 5px;"
                               % ("#22c55e" if ok else "#c9ccdc"))
        self.status_lbl.setText(text)

    def _build_topbar(self):
        wrap = QWidget()
        h = QHBoxLayout(wrap)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(12)

        conn, cl = make_card("串口连接")
        row = QHBoxLayout()
        row.setSpacing(8)
        self.cmb_port = QComboBox()
        self.cmb_port.setMinimumWidth(190)
        self.cmb_port.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.setFixedWidth(60)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_conn = QPushButton("连接")
        self.btn_conn.setObjectName("Accent")
        self.btn_conn.setFixedWidth(84)
        self.btn_conn.clicked.connect(self.toggle_conn)
        row.addWidget(self.cmb_port, 1)
        row.addWidget(self.btn_refresh)
        row.addWidget(self.btn_conn)
        cl.addLayout(row)
        conn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        h.addWidget(conn, 3)

        pre, pl = make_card("预设")
        prow = QHBoxLayout()
        prow.setSpacing(6)
        self.cmb_preset = QComboBox()
        self.cmb_preset.setEditable(True)
        self.cmb_preset.setMinimumWidth(120)
        self.cmb_preset.lineEdit().setPlaceholderText("预设名称")
        prow.addWidget(self.cmb_preset, 1)
        for text, slot in (("加载", self.preset_load), ("保存", self.preset_save),
                           ("删除", self.preset_delete), ("导入", self.preset_import),
                           ("导出", self.preset_export)):
            b = QPushButton(text)
            b.setFixedWidth(48)
            b.clicked.connect(slot)
            prow.addWidget(b)
        pl.addLayout(prow)
        pre.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        h.addWidget(pre, 4)
        return wrap

    def _build_pages(self):
        for maker in (self._page_effect, self._page_color, self._page_seg,
                      self._page_pixel, self._page_play, self._page_music,
                      self._page_strips):
            self.stack.addWidget(maker())

    def _build_log(self, parent_layout):
        self.log_card, cl = make_card("日志")
        self.log_box = QPlainTextEdit()
        self.log_box.setObjectName("Log")
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(800)
        self.log_box.setFixedHeight(112)
        cl.addWidget(self.log_box)
        parent_layout.addWidget(self.log_card)

    def _toggle_log(self, on):
        self.log_card.setVisible(on)

    def _switch_page(self, idx):
        if idx == self.stack.currentIndex():
            return
        prev = self.stack.currentWidget()
        if prev is not None:
            prev.setGraphicsEffect(None)       # 清掉上一页可能残留的透明度效果
        self.stack.setCurrentIndex(idx)
        w = self.stack.currentWidget()
        if w is None:
            return
        eff = QGraphicsOpacityEffect(w)
        w.setGraphicsEffect(eff)
        anim = QPropertyAnimation(eff, b"opacity", w)
        anim.setDuration(180)
        anim.setStartValue(0.0)
        anim.setEndValue(1.0)
        anim.setEasingCurve(QEasingCurve.OutCubic)
        anim.finished.connect(lambda: self._clear_effect(w))
        anim.start(QAbstractAnimation.DeleteWhenStopped)

    @staticmethod
    def _clear_effect(w):
        try:
            w.setGraphicsEffect(None)
        except RuntimeError:
            pass

    # ------------------------------------------------------------ 通用控件
    def _slider_row(self, grid, row, col, text, frm, to, init, cmd, key):
        w = QWidget()
        h = QHBoxLayout(w)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(10)
        lbl = QLabel(text)
        lbl.setObjectName("Field")
        lbl.setFixedWidth(58)
        sl = QSlider(Qt.Horizontal)
        sl.setRange(frm, to)
        sl.setValue(init)
        sl.setMinimumWidth(110)
        val = QLabel(str(init))
        val.setObjectName("Value")
        val.setFixedWidth(32)
        val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        h.addWidget(lbl)
        h.addWidget(sl, 1)
        h.addWidget(val)
        grid.addWidget(w, row, col)
        sl.valueChanged.connect(
            lambda v, c=cmd, k=key, vl=val: self._on_slider(c, k, v, vl))
        return sl, val

    # ------------------------------------------------------------ 分页 1 灯效
    def _page_effect(self):
        p = QWidget()
        v = QVBoxLayout(p)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(12)

        card, cl = make_card("灯效模式")
        grid = QGridLayout()
        grid.setSpacing(8)
        self.mode_btns = {}
        for idx, (name, val) in enumerate(MODES):
            b = QPushButton(name)
            b.setObjectName("Mode")
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.setMinimumHeight(34)
            b.clicked.connect(lambda _=False, x=val: self.set_mode(x))
            grid.addWidget(b, idx // 5, idx % 5)
            self.mode_btns[val] = b
        for c in range(5):
            grid.setColumnStretch(c, 1)
        cl.addLayout(grid)
        v.addWidget(card)

        card2, cl2 = make_card("亮度 / 速度 / 高级参数")
        g = QGridLayout()
        g.setHorizontalSpacing(24)
        g.setVerticalSpacing(9)
        self.sl_bright, self.lb_bright = self._slider_row(
            g, 0, 0, "亮度", 0, 255, 120, "BRIGHT", "bright")
        self.sl_speed, self.lb_speed = self._slider_row(
            g, 0, 1, "速度", 1, 100, 50, "SPEED", "speed")
        self.sl_random, self.lb_random = self._slider_row(
            g, 1, 0, "随机强度", 0, 100, 20, "RANDOM", "random")
        self.sl_gamma, self.lb_gamma = self._slider_row(
            g, 1, 1, "Gamma", 0, 100, 0, "GAMMA", "gamma")
        self.sl_tail, self.lb_tail = self._slider_row(
            g, 2, 0, "尾迹长度", 1, LED_COUNT, 6, "TAIL", "tail")
        self.sl_spacing, self.lb_spacing = self._slider_row(
            g, 2, 1, "灯间距", 0, 6, 1, "SPACING", "spacing")
        g.setColumnStretch(0, 1)
        g.setColumnStretch(1, 1)
        cl2.addLayout(g)
        v.addWidget(card2)

        card3, cl3 = make_card("开关")
        r = QHBoxLayout()
        r.setSpacing(28)
        self.ck_dir = QCheckBox("反向")
        self.ck_mirror = QCheckBox("镜像对称")
        self.ck_sync = QCheckBox("板载 GPIO8 同步")
        self.ck_dir.toggled.connect(lambda x: self._on_flag("DIR", "dir", x))
        self.ck_mirror.toggled.connect(lambda x: self._on_flag("MIRROR", "mirror", x))
        self.ck_sync.toggled.connect(lambda x: self._on_flag("SYNC", "sync", x))
        for w in (self.ck_dir, self.ck_mirror, self.ck_sync):
            r.addWidget(w)
        r.addStretch(1)
        cl3.addLayout(r)
        v.addWidget(card3)
        v.addStretch(1)
        return p

    # ------------------------------------------------------------ 分页 2 颜色
    def _page_color(self):
        p = QWidget()
        v = QVBoxLayout(p)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(12)

        card, cl = make_card("起色 / 终色")
        g = QGridLayout()
        g.setHorizontalSpacing(12)
        g.setVerticalSpacing(10)

        l1 = QLabel("起色")
        l1.setObjectName("Field")
        l1.setFixedWidth(44)
        self.sw1 = Swatch((0, 150, 255))
        self.lbl_c1 = QLabel("#0096FF")
        self.lbl_c1.setObjectName("Mono")
        self.lbl_c1.setFixedWidth(84)
        b1 = QPushButton("选择颜色")
        b1.setFixedWidth(84)
        b1.clicked.connect(self.pick_color1)

        l2 = QLabel("终色")
        l2.setObjectName("Field")
        l2.setFixedWidth(44)
        self.sw2 = Swatch((0, 0, 0))
        self.lbl_c2 = QLabel("#000000")
        self.lbl_c2.setObjectName("Mono")
        self.lbl_c2.setFixedWidth(84)
        b2 = QPushButton("选择颜色")
        b2.setFixedWidth(84)
        b2.clicked.connect(self.pick_color2)

        g.addWidget(l1, 0, 0)
        g.addWidget(self.sw1, 0, 1)
        g.addWidget(self.lbl_c1, 0, 2)
        g.addWidget(b1, 0, 3)
        g.addWidget(l2, 1, 0)
        g.addWidget(self.sw2, 1, 1)
        g.addWidget(self.lbl_c2, 1, 2)
        g.addWidget(b2, 1, 3)
        g.setColumnStretch(4, 1)
        cl.addLayout(g)
        v.addWidget(card)

        card2, cl2 = make_card("渐变预览")
        self.preview = GradientPreview()
        cl2.addWidget(self.preview)
        v.addWidget(card2)

        card3, cl3 = make_card("色相跨度（彩虹 / 流光展开范围）")
        g3 = QGridLayout()
        self.sl_hue, self.lb_hue = self._slider_row(
            g3, 0, 0, "色相跨度", 0, 255, 255, "HUE", "hueSpan")
        g3.setColumnStretch(0, 1)
        cl3.addLayout(g3)
        v.addWidget(card3)
        v.addStretch(1)
        return p

    # ------------------------------------------------------------ 分页 3 分段
    def _page_seg(self):
        p = QWidget()
        v = QVBoxLayout(p)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(12)

        card, cl = make_card("分段列表（存在分段时优先于全局灯效）")
        self.lst_seg = QListWidget()
        self.lst_seg.setMinimumHeight(150)
        self.lst_seg.currentRowChanged.connect(self._seg_select)
        cl.addWidget(self.lst_seg)
        v.addWidget(card, 1)

        card2, cl2 = make_card("段编辑")
        g = QGridLayout()
        g.setHorizontalSpacing(10)
        g.setVerticalSpacing(8)

        def fl(t):
            lb = QLabel(t)
            lb.setObjectName("Field")
            return lb

        self.sp_seg_start = QSpinBox()
        self.sp_seg_start.setRange(0, LED_COUNT - 1)
        self.sp_seg_start.setFixedWidth(62)
        self.sp_seg_len = QSpinBox()
        self.sp_seg_len.setRange(1, LED_COUNT)
        self.sp_seg_len.setValue(5)
        self.sp_seg_len.setFixedWidth(62)
        self.cmb_seg_mode = QComboBox()
        self.cmb_seg_mode.addItems(MODE_NAMES)
        self.cmb_seg_mode.setCurrentText("呼吸")
        self.cmb_seg_mode.setFixedWidth(96)
        self.sw_seg = Swatch((255, 0, 0), (44, 28))
        bsw = QPushButton("选色")
        bsw.setFixedWidth(56)
        bsw.clicked.connect(self._seg_pick)

        g.addWidget(fl("起始"), 0, 0)
        g.addWidget(self.sp_seg_start, 0, 1)
        g.addWidget(fl("长度"), 0, 2)
        g.addWidget(self.sp_seg_len, 0, 3)
        g.addWidget(fl("模式"), 0, 4)
        g.addWidget(self.cmb_seg_mode, 0, 5)
        g.addWidget(self.sw_seg, 0, 6)
        g.addWidget(bsw, 0, 7)
        g.setColumnStretch(8, 1)
        cl2.addLayout(g)
        v.addWidget(card2)

        bar = QWidget()
        hb = QHBoxLayout(bar)
        hb.setContentsMargins(0, 0, 0, 0)
        hb.setSpacing(8)
        for text, slot in (("新增", self.seg_add), ("更新选中", self.seg_update),
                           ("删除选中", self.seg_delete), ("清空全部", self.seg_clear)):
            bb = QPushButton(text)
            bb.clicked.connect(slot)
            hb.addWidget(bb)
        hb.addStretch(1)
        v.addWidget(bar)
        return p

    # ------------------------------------------------------------ 分页 4 像素画
    def _page_pixel(self):
        p = QWidget()
        v = QVBoxLayout(p)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(12)

        card, cl = make_card("逐颗上色（点击格子涂当前画笔色；灯珠较多时可横向滚动）")
        self.cv_pix = PixelCanvas(count=LED_COUNT)
        self.cv_pix.cellClicked.connect(self._pix_click)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.cv_pix)
        scroll.setFixedHeight(92)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.NoFrame)
        cl.addWidget(scroll)
        v.addWidget(card)

        card2, cl2 = make_card("调色板 / 画笔")
        r = QHBoxLayout()
        r.setSpacing(8)
        for hx in PALETTE:
            b = QPushButton()
            b.setFixedSize(26, 26)
            b.setCursor(Qt.PointingHandCursor)
            b.setStyleSheet(
                "QPushButton{background:%s;border:1px solid rgba(0,0,0,45);border-radius:13px;}"
                "QPushButton:hover{border:2px solid #667eea;}" % hx)
            b.clicked.connect(lambda _=False, c=hx: self._set_brush(c))
            r.addWidget(b)
        self.lbl_brush = QLabel("画笔 #FF0000")
        self.lbl_brush.setObjectName("Mono")
        r.addSpacing(8)
        r.addWidget(self.lbl_brush)
        bc = QPushButton("自定义")
        bc.setFixedWidth(64)
        bc.clicked.connect(self._brush_pick)
        r.addWidget(bc)
        r.addStretch(1)
        cl2.addLayout(r)
        v.addWidget(card2)

        bar = QWidget()
        hb = QHBoxLayout(bar)
        hb.setContentsMargins(0, 0, 0, 0)
        hb.setSpacing(8)
        bf = QPushButton("填充全部")
        bf.clicked.connect(self.pix_fill)
        bcl = QPushButton("清空")
        bcl.setFixedWidth(70)
        bcl.clicked.connect(self.pix_clear)
        ba = QPushButton("应用并显示")
        ba.setObjectName("Accent")
        ba.setFixedWidth(110)
        ba.clicked.connect(self.pix_apply)
        hb.addWidget(bf)
        hb.addWidget(bcl)
        hb.addWidget(ba)
        hb.addStretch(1)
        v.addWidget(bar)
        v.addStretch(1)
        return p

    # ------------------------------------------------------------ 分页 7 灯带配置
    def _page_strips(self):
        p = QWidget()
        v = QVBoxLayout(p)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(12)

        card, cl = make_card("灯带通道（灯珠数量未知也没关系，按实际填写即可）")

        head = QGridLayout()
        head.setHorizontalSpacing(20)
        for col, txt in enumerate(("通道", "启用", "GPIO", "灯珠数量")):
            lb = QLabel(txt)
            lb.setObjectName("Field")
            head.addWidget(lb, 0, col)
        head.setColumnStretch(4, 1)
        cl.addLayout(head)

        grid = QGridLayout()
        grid.setHorizontalSpacing(20)
        grid.setVerticalSpacing(8)
        self.strip_rows = []
        for i in range(MAX_STRIPS):
            lb = QLabel("第 %d 路" % (i + 1))
            lb.setObjectName("Field")
            ck = QCheckBox()
            ck.setChecked(i == 0)
            pin = QComboBox()
            pin.addItems([str(g) for g in GPIO_CHOICES])
            pin.setCurrentText("10")
            pin.setFixedWidth(72)
            cnt = QSpinBox()
            cnt.setRange(1, MAX_PER_STRIP)
            cnt.setValue(LED_COUNT if i == 0 else 1)
            cnt.setFixedWidth(84)
            grid.addWidget(lb, i, 0)
            grid.addWidget(ck, i, 1)
            grid.addWidget(pin, i, 2)
            grid.addWidget(cnt, i, 3)
            self.strip_rows.append((ck, pin, cnt))
        grid.setColumnStretch(4, 1)
        cl.addLayout(grid)

        row = QHBoxLayout()
        row.setSpacing(10)
        lm = QLabel("各路关系")
        lm.setObjectName("Field")
        self.cmb_merge = QComboBox()
        self.cmb_merge.addItem("拼接成一条长灯带", 1)
        self.cmb_merge.addItem("各路独立跑同一灯效", 0)
        self.cmb_merge.setFixedWidth(180)
        self.lbl_total = QLabel("")
        self.lbl_total.setObjectName("Hint")
        row.addWidget(lm)
        row.addWidget(self.cmb_merge)
        row.addSpacing(12)
        row.addWidget(self.lbl_total)
        row.addStretch(1)
        cl.addLayout(row)
        v.addWidget(card)

        bar = QWidget()
        hb = QHBoxLayout(bar)
        hb.setContentsMargins(0, 0, 0, 0)
        hb.setSpacing(8)
        ba = QPushButton("应用灯带配置")
        ba.setObjectName("Accent")
        ba.setFixedWidth(130)
        ba.clicked.connect(self.strips_apply)
        br = QPushButton("恢复默认")
        br.setFixedWidth(90)
        br.clicked.connect(self.strips_reset)
        hb.addWidget(ba)
        hb.addWidget(br)
        hb.addStretch(1)
        v.addWidget(bar)

        hint = QLabel("可用 GPIO：0~7、10（8/9/11 及以上被板载 LED、BOOT、Flash、USB 占用）。"
                      "单路上限 %d 颗，逻辑总长上限 %d 颗。" % (MAX_PER_STRIP, MAX_TOTAL_PIXELS))
        hint.setObjectName("Hint")
        v.addWidget(hint)
        v.addStretch(1)
        return p

    @staticmethod
    def _logical_from_cfg(cfg):
        tot = 0
        mx = 0
        for i in range(min(cfg.get("numStrips", 1), MAX_STRIPS)):
            d = cfg["strips"][i]
            c = d["count"] if d["enabled"] else 0
            tot += c
            mx = max(mx, c)
        n = tot if cfg.get("merge") else mx
        return max(1, min(MAX_TOTAL_PIXELS, n)) if n else 0

    def _apply_logical_len(self, n):
        n = max(1, min(MAX_TOTAL_PIXELS, n))
        if n != len(self.pix):
            old = self.pix
            if len(old) < n:
                self.pix = old + [(0, 0, 0)] * (n - len(old))
            else:
                self.pix = old[:n]
            self.cv_pix.set_count(n)
            self.cv_pix.set_pixels(self.pix)
        self.sp_seg_start.setRange(0, n - 1)
        self.sp_seg_len.setRange(1, n)
        self.sl_tail.setMaximum(n)
        self.logical_len = n
        self._refresh_strip_ui()

    def _refresh_strip_ui(self):
        cfg = self.strip_cfg
        for i, (ck, pin, cnt) in enumerate(self.strip_rows):
            d = cfg["strips"][i]
            ck.setChecked(bool(d["enabled"]))
            j = pin.findText(str(d["pin"]))
            if j >= 0:
                pin.setCurrentIndex(j)
            cnt.setValue(max(1, min(MAX_PER_STRIP, d["count"])))
        i = self.cmb_merge.findData(1 if cfg.get("merge") else 0)
        if i >= 0:
            self.cmb_merge.setCurrentIndex(i)
        tot = 0
        mx = 0
        for i in range(min(cfg.get("numStrips", 1), MAX_STRIPS)):
            d = cfg["strips"][i]
            c = d["count"] if d["enabled"] else 0
            tot += c
            mx = max(mx, c)
        logic = tot if cfg.get("merge") else mx
        self.lbl_total.setText("逻辑长度 %d 颗（%s）" % (
            logic, "拼接" if cfg.get("merge") else "独立"))
        self.lbl_sub.setText("ESP32-C3 · WS2812B ×%d%s" % (
            logic, " ·%d路" % cfg.get("numStrips", 1) if cfg.get("numStrips", 1) > 1 else ""))

    def strips_apply(self):
        num = 0
        for i, (ck, _pin, _cnt) in enumerate(self.strip_rows):
            if ck.isChecked():
                num = i + 1
        if num == 0:
            self.log("至少需要启用一路灯带")
            return
        merge = 1 if self.cmb_merge.currentData() == 1 else 0
        tot = 0
        mx = 0
        for i in range(num):
            ck, _pin, cnt = self.strip_rows[i]
            c = cnt.value() if ck.isChecked() else 0
            tot += c
            mx = max(mx, c)
        logic = tot if merge else mx
        if logic > MAX_TOTAL_PIXELS:
            self.log("灯珠总数 %d 超过上限 %d，请调小" % (logic, MAX_TOTAL_PIXELS))
            return
        self.strip_cfg = {
            "numStrips": num,
            "merge": merge,
            "strips": [{"pin": int(pin.currentText()), "count": cnt.value(),
                        "enabled": 1 if ck.isChecked() else 0}
                       for (ck, pin, cnt) in self.strip_rows],
        }
        self.send("STRIPS %d %d" % (num, merge))
        for i in range(num):
            ck, pin, cnt = self.strip_rows[i]
            self.send("STRIPSET %d %d %d %d" % (
                i, int(pin.currentText()), cnt.value(), 1 if ck.isChecked() else 0))
        self._apply_logical_len(logic if logic else 1)
        self.log("已应用灯带配置：%d 路，%s，逻辑长度 %d" % (
            num, "拼接" if merge else "独立", logic))

    def strips_reset(self):
        if not self.connected:
            self.log("未连接，无法恢复默认")
            return
        self.send("STRIPCLR")
        self.strip_cfg = {
            "numStrips": 1,
            "merge": 1,
            "strips": [{"pin": 10, "count": LED_COUNT, "enabled": 1 if i == 0 else 0}
                       for i in range(MAX_STRIPS)],
        }
        self._apply_logical_len(LED_COUNT)
        self.log("已恢复默认灯带：1 路 GPIO10 / %d 颗 / 拼接" % LED_COUNT)

    # ------------------------------------------------------------ 分页 5 编排
    def _page_play(self):
        p = QWidget()
        v = QVBoxLayout(p)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(12)

        card, cl = make_card("编排步骤（按顺序轮播）")
        self.lst_step = QListWidget()
        self.lst_step.setMinimumHeight(150)
        self.lst_step.currentRowChanged.connect(self._step_select)
        cl.addWidget(self.lst_step)
        v.addWidget(card, 1)

        card2, cl2 = make_card("步骤编辑")
        g = QGridLayout()
        g.setHorizontalSpacing(10)
        g.setVerticalSpacing(8)

        lm = QLabel("模式")
        lm.setObjectName("Field")
        self.cmb_step_mode = QComboBox()
        self.cmb_step_mode.addItems(MODE_NAMES)
        self.cmb_step_mode.setCurrentText("呼吸")
        self.cmb_step_mode.setFixedWidth(96)
        ls = QLabel("时长(秒)")
        ls.setObjectName("Field")
        self.sp_step_secs = QSpinBox()
        self.sp_step_secs.setRange(1, 3600)
        self.sp_step_secs.setValue(5)
        self.sp_step_secs.setFixedWidth(72)

        self.sw_step1 = Swatch(self.step_c1, (40, 26))
        b1 = QPushButton("起色")
        b1.setFixedWidth(52)
        b1.clicked.connect(lambda: self._step_pick(1))
        self.sw_step2 = Swatch(self.step_c2, (40, 26))
        b2 = QPushButton("终色")
        b2.setFixedWidth(52)
        b2.clicked.connect(lambda: self._step_pick(2))

        g.addWidget(lm, 0, 0)
        g.addWidget(self.cmb_step_mode, 0, 1)
        g.addWidget(ls, 0, 2)
        g.addWidget(self.sp_step_secs, 0, 3)
        g.addWidget(b1, 1, 0)
        g.addWidget(self.sw_step1, 1, 1)
        g.addWidget(b2, 1, 2)
        g.addWidget(self.sw_step2, 1, 3)
        g.setColumnStretch(4, 1)
        cl2.addLayout(g)
        v.addWidget(card2)

        bar = QWidget()
        hb = QHBoxLayout(bar)
        hb.setContentsMargins(0, 0, 0, 0)
        hb.setSpacing(8)
        for text, slot in (("新增", self.step_add), ("更新选中", self.step_update),
                           ("删除选中", self.step_delete), ("清空", self.step_clear)):
            bb = QPushButton(text)
            bb.clicked.connect(slot)
            hb.addWidget(bb)
        hb.addSpacing(16)
        bp = QPushButton("开始轮播")
        bp.setObjectName("Accent")
        bp.setFixedWidth(96)
        bp.clicked.connect(self.play_start)
        bs = QPushButton("停止")
        bs.setFixedWidth(64)
        bs.clicked.connect(self.play_stop)
        bn = QPushButton("下一步")
        bn.setFixedWidth(76)
        bn.clicked.connect(self.play_next)
        hb.addWidget(bp)
        hb.addWidget(bs)
        hb.addWidget(bn)
        hb.addStretch(1)
        v.addWidget(bar)
        return p

    # ------------------------------------------------------------ 分页 6 音乐
    def _page_music(self):
        p = QWidget()
        v = QVBoxLayout(p)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(12)

        card, cl = make_card("音频来源")
        r = QHBoxLayout()
        r.setSpacing(8)
        self.cmb_dev = QComboBox()
        self.cmb_dev.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        br = QPushButton("刷新设备")
        br.setFixedWidth(84)
        br.clicked.connect(self.refresh_devices)
        r.addWidget(self.cmb_dev, 1)
        r.addWidget(br)
        cl.addLayout(r)
        tip = QLabel("「系统声音」直接采集扬声器输出（回环），跟随电脑实际播放的音量律动；"
                     "「麦克风」采集环境声音。默认优先选中当前扬声器的系统声音。")
        tip.setObjectName("Hint")
        tip.setWordWrap(True)
        cl.addWidget(tip)
        v.addWidget(card)

        card2, cl2 = make_card("律动模式 / 无声回落")
        g2 = QGridLayout()
        g2.setHorizontalSpacing(16)
        g2.setVerticalSpacing(9)
        lbl_rm = QLabel("律动模式")
        lbl_rm.setObjectName("Field")
        g2.addWidget(lbl_rm, 0, 0)
        self.cmb_rmode = QComboBox()
        self.cmb_rmode.addItem("音乐律动（整体）", 14)
        self.cmb_rmode.addItem("立体声律动（左右声道）", 15)
        g2.addWidget(self.cmb_rmode, 0, 1)
        lbl_idle = QLabel("无声时灯效")
        lbl_idle.setObjectName("Field")
        g2.addWidget(lbl_idle, 1, 0)
        self.cmb_idle = QComboBox()
        for name, val in MODES:
            if val != 15:
                self.cmb_idle.addItem(name, val)
        self.cmb_idle.setCurrentIndex(self.cmb_idle.findData(2))
        self.cmb_idle.currentIndexChanged.connect(self.set_idle)
        g2.addWidget(self.cmb_idle, 1, 1)
        g2.setColumnStretch(1, 1)
        cl2.addLayout(g2)
        tip2 = QLabel("立体声律动：左半屏跟左声道、右半屏跟右声道，各自从两端向中间律动；"
                      "安静时自动回落为上面选定的「无声灯效」。")
        tip2.setObjectName("Hint")
        tip2.setWordWrap(True)
        cl2.addWidget(tip2)
        v.addWidget(card2)

        card3, cl3 = make_card("灵敏度")
        g = QGridLayout()
        self.sl_gain, self.lb_gain = self._slider_row(
            g, 0, 0, "灵敏度", 0, 100, 0, "GAIN", "gain")
        g.setColumnStretch(0, 1)
        cl3.addLayout(g)
        v.addWidget(card3)

        card4, cl4 = make_card("实时电平")
        self.meter = LevelMeter()
        cl4.addWidget(self.meter)
        self.stereo_meter = StereoMeter()
        cl4.addWidget(self.stereo_meter)
        v.addWidget(card4)

        bar = QWidget()
        hb = QHBoxLayout(bar)
        hb.setContentsMargins(0, 0, 0, 0)
        hb.setSpacing(10)
        self.btn_music = QPushButton("启动律动")
        self.btn_music.setObjectName("Accent")
        self.btn_music.setFixedWidth(100)
        self.btn_music.clicked.connect(self.toggle_music)
        hb.addWidget(self.btn_music)
        hint = QLabel("系统声音需 pip install soundcard numpy")
        hint.setObjectName("Hint")
        hb.addWidget(hint)
        hb.addStretch(1)
        v.addWidget(bar)
        if not AUDIO_OK:
            warn = QLabel("当前未安装音频依赖，此分页不可用（pip install soundcard numpy）")
            warn.setStyleSheet("color: #cc3333; font-size: 11px;")
            v.addWidget(warn)
        v.addStretch(1)
        return p

    # ------------------------------------------------------------ 串口
    def refresh_ports(self):
        self.cmb_port.clear()
        items = []
        for p in list_ports.comports():
            desc = p.description or ""
            items.append("%s  %s" % (p.device, desc) if desc else p.device)
        self.cmb_port.addItems(items)
        for i, t in enumerate(items):
            if any(k in t.upper() for k in ("USB", "CH340", "CP210", "JTAG")):
                self.cmb_port.setCurrentIndex(i)
                break
        self.log("发现串口 %d 个" % len(items))

    def _cur_port(self):
        t = self.cmb_port.currentText().strip()
        return t.split()[0] if t else ""

    def toggle_conn(self):
        if self.connected or self.thread:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self._cur_port()
        if not port:
            self.log("请先选择串口")
            return
        self._conn_port = port
        self.thread = QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.thread)
        self.worker.configure(port, BAUD)
        self.thread.started.connect(self.worker.run)
        self.worker.lineReceived.connect(self._on_line)
        self.worker.opened.connect(self._on_opened)
        self.worker.closed.connect(self._on_closed)
        self.worker.failed.connect(self._on_failed)
        self.thread.start()

    def disconnect(self):
        if self.music_run:
            self.music_stop()
        worker, th = self.worker, self.thread
        self.worker = None
        self.thread = None
        if worker:
            worker.stop()
        if th is not None:
            th.quit()
            if not th.wait(1500):
                th.terminate()
                th.wait(500)
        self._on_closed()

    def _on_opened(self):
        self.connected = True
        self._synced = False
        self.btn_conn.setText("断开")
        self._set_status(True, "已连接 %s" % self._conn_port)
        self.settings.setValue("port", self._conn_port)      # 记住默认端口
        if self.tray:
            self.tray.setToolTip("ESP32-C3 神光同步工作台 — 已连接 %s" % self._conn_port)
        if self.act_conn:
            self.act_conn.setText("断开串口")
        self.log("已连接 %s @ %d" % (self._conn_port, BAUD))
        QTimer.singleShot(400, lambda: self.send("PING"))
        # 打开串口可能触发板子复位，启动完成前发的 GET 会丢失，这里重试几次
        for delay in (700, 1300, 2100, 3000):
            QTimer.singleShot(delay, self._request_state)

    def _request_state(self):
        if self.connected and not self._synced:
            self.send("GET", quiet=True)

    def _on_closed(self):
        was = self.connected
        self.connected = False
        self.btn_conn.setText("连接")
        self._set_status(False, "未连接")
        if self.tray:
            self.tray.setToolTip("ESP32-C3 神光同步工作台 — 未连接")
        if self.act_conn:
            self.act_conn.setText("连接串口")
        if was:
            self.log("已断开")

    def _on_failed(self, msg):
        self.log("错误: %s" % msg)

    def send(self, cmd, quiet=False):
        if not (self.worker and self.connected):
            if not quiet:
                self.log("未连接，忽略: %s" % cmd)
            return
        self.worker.enqueue(cmd)
        if not quiet:
            self.log(">> " + cmd)

    def _on_line(self, text):
        self.log("<< " + text)
        up = text.upper()
        if up.startswith("STATE "):
            self._pending = {"state": text}
        elif up.startswith("SEG "):
            self._pending.setdefault("segs", []).append(text)
        elif up.startswith("STEP "):
            self._pending.setdefault("steps", []).append(text)
        elif up.startswith("STRIPS "):
            self._pending["strips_meta"] = text
        elif up.startswith("STRIP "):
            self._pending.setdefault("strip_rows", []).append(text)
        elif up.startswith("PIX "):
            self._pending.setdefault("pix", []).append(text)
        elif up == "END" and self._pending.get("state"):
            data = self._pending
            self._pending = {}
            self.apply_block(data)

    # ------------------------------------------------------------ 状态同步
    def apply_block(self, data):
        try:
            v = [int(x) for x in data["state"].split()[1:23]]
        except Exception:
            return
        if len(v) < 21:
            return
        if len(v) < 22:
            v.append(2)                      # 旧固件没有 idleMode，补默认呼吸
        self._synced = True
        self.syncing = True
        try:
            s = self.state
            (s["mode"], s["r1"], s["g1"], s["b1"], s["bright"], s["speed"], s["sync"],
             s["r2"], s["g2"], s["b2"], s["hueSpan"], s["dir"], s["tail"], s["spacing"],
             s["mirror"], s["random"], s["gamma"], s["gain"],
             s["segCount"], s["stepCount"], s["playing"], s["idle"]) = v

            self.segs = []
            for line in data.get("segs", []):
                p = line.split()
                if len(p) >= 8:
                    self.segs.append({"start": int(p[2]), "length": int(p[3]),
                                      "mode": int(p[4]), "r": int(p[5]),
                                      "g": int(p[6]), "b": int(p[7])})
            self.steps = []
            for line in data.get("steps", []):
                p = line.split()
                if len(p) >= 10:
                    self.steps.append({"mode": int(p[2]), "secs": int(p[3]),
                                       "r1": int(p[4]), "g1": int(p[5]), "b1": int(p[6]),
                                       "r2": int(p[7]), "g2": int(p[8]), "b2": int(p[9])})
            self._parse_strips(data)
            self.pix = [(0, 0, 0)] * self.logical_len
            for line in data.get("pix", []):
                p = line.split()
                if len(p) >= 5 and 0 <= int(p[1]) < self.logical_len:
                    self.pix[int(p[1])] = (int(p[2]), int(p[3]), int(p[4]))

            self._refresh_all()
        finally:
            self.syncing = False
        self.log("[已同步板子当前状态]")
        # 板子当前就是律动灯效（音乐/立体声）-> 直接开始律动，无需手动点「启动律动」
        if self.state["mode"] in (14, 15) and not self.music_run:
            idx = self.cmb_rmode.findData(self.state["mode"])
            if idx >= 0:
                self.cmb_rmode.setCurrentIndex(idx)
            self.music_start(auto=True)

    def _parse_strips(self, data):
        """解析固件回传的 STRIPS / STRIP 行，刷新灯带配置与逻辑长度。"""
        meta = data.get("strips_meta", "")
        if not meta:
            return
        p = meta.split()
        if len(p) < 4:
            return
        num = max(1, min(MAX_STRIPS, int(p[1])))
        merge = 1 if int(p[2]) else 0
        logical = max(0, int(p[3]))
        rows = [{"pin": 10, "count": LED_COUNT, "enabled": 0} for _ in range(MAX_STRIPS)]
        for line in data.get("strip_rows", []):
            q = line.split()
            if len(q) >= 5 and 0 <= int(q[1]) < MAX_STRIPS:
                rows[int(q[1])] = {"pin": int(q[2]), "count": int(q[3]),
                                   "enabled": int(q[4])}
        if not any(r["enabled"] for r in rows):
            rows[0] = {"pin": 10, "count": LED_COUNT, "enabled": 1}
        self.strip_cfg = {"numStrips": num, "merge": merge, "strips": rows}
        self._apply_logical_len(logical if logical else self._logical_from_cfg(self.strip_cfg))

    def _refresh_all(self):
        s = self.state
        self._refresh_strip_ui()
        self._highlight_mode()
        self.sw1.set_color((s["r1"], s["g1"], s["b1"]))
        self.lbl_c1.setText(rgb2hex((s["r1"], s["g1"], s["b1"])))
        self.sw2.set_color((s["r2"], s["g2"], s["b2"]))
        self.lbl_c2.setText(rgb2hex((s["r2"], s["g2"], s["b2"])))
        for slider, var, key, lbl in (
                (self.sl_bright, None, "bright", self.lb_bright),
                (self.sl_speed, None, "speed", self.lb_speed),
                (self.sl_random, None, "random", self.lb_random),
                (self.sl_gamma, None, "gamma", self.lb_gamma),
                (self.sl_tail, None, "tail", self.lb_tail),
                (self.sl_spacing, None, "spacing", self.lb_spacing),
                (self.sl_hue, None, "hueSpan", self.lb_hue),
                (self.sl_gain, None, "gain", self.lb_gain)):
            slider.setValue(s[key])
            lbl.setText(str(s[key]))
        self.ck_dir.setChecked(bool(s["dir"]))
        self.ck_mirror.setChecked(bool(s["mirror"]))
        self.ck_sync.setChecked(bool(s["sync"]))
        i = self.cmb_idle.findData(s["idle"])
        if i >= 0:
            self.cmb_idle.setCurrentIndex(i)
        if s["mode"] in (14, 15):
            i = self.cmb_rmode.findData(s["mode"])
            if i >= 0:
                self.cmb_rmode.setCurrentIndex(i)
        self._seg_refresh_list()
        self._step_refresh_list()
        self._draw_gradient()
        self.cv_pix.set_pixels(self.pix)

    def _highlight_mode(self):
        for val, btn in self.mode_btns.items():
            btn.setChecked(val == self.state["mode"])

    def _draw_gradient(self):
        s = self.state
        self.preview.set_colors((s["r1"], s["g1"], s["b1"]),
                                (s["r2"], s["g2"], s["b2"]))

    # ------------------------------------------------------------ 回调
    def set_mode(self, val):
        if self.syncing:
            return
        self.state["mode"] = val
        self._highlight_mode()
        if self.segs:
            self.segs = []
            self._seg_refresh_list()
            self.send("SEGCLR")
        self.send("PLAY 0")
        self.send("MODE %d" % val)

    def pick_color1(self):
        s = self.state
        c = QColorDialog.getColor(QColor(s["r1"], s["g1"], s["b1"]), self, "选择起色")
        if c.isValid():
            self._set_color1((c.red(), c.green(), c.blue()))

    def pick_color2(self):
        s = self.state
        c = QColorDialog.getColor(QColor(s["r2"], s["g2"], s["b2"]), self, "选择终色")
        if c.isValid():
            self._set_color2((c.red(), c.green(), c.blue()))

    def _set_color1(self, c):
        s = self.state
        s["r1"], s["g1"], s["b1"] = c
        self.sw1.set_color(c)
        self.lbl_c1.setText(rgb2hex(c))
        self._draw_gradient()
        self.send("COLOR %d %d %d" % c)

    def _set_color2(self, c):
        s = self.state
        s["r2"], s["g2"], s["b2"] = c
        self.sw2.set_color(c)
        self.lbl_c2.setText(rgb2hex(c))
        self._draw_gradient()
        self.send("COLOR2 %d %d %d" % c)

    def _on_slider(self, cmd, key, value, label):
        value = int(value)
        label.setText(str(value))
        if self.syncing:
            return
        self.state[key] = value
        if key == "hueSpan":
            self._draw_gradient()
        timer = self._deb.get(cmd)
        if timer is None:
            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.setInterval(80)
            timer.timeout.connect(
                lambda c=cmd, k=key: self.send("%s %d" % (c, self.state[k])))
            self._deb[cmd] = timer
        timer.start()

    def _on_flag(self, cmd, key, checked):
        if self.syncing:
            return
        self.state[key] = 1 if checked else 0
        self.send("%s %d" % (cmd, self.state[key]))

    # ------------------------------------------------------------ 分段
    def _seg_refresh_list(self):
        self.lst_seg.clear()
        for i, g in enumerate(self.segs):
            self.lst_seg.addItem("段%d    起 %2d   长 %2d    %s    %s" % (
                i, g["start"], g["length"],
                MODE_NAME_BY_VAL.get(g["mode"], "?"),
                rgb2hex((g["r"], g["g"], g["b"]))))

    def _seg_select(self, row):
        if row < 0 or row >= len(self.segs):
            return
        g = self.segs[row]
        self.sp_seg_start.setValue(g["start"])
        self.sp_seg_len.setValue(g["length"])
        self.cmb_seg_mode.setCurrentText(MODE_NAME_BY_VAL.get(g["mode"], "呼吸"))
        self.seg_color = (g["r"], g["g"], g["b"])
        self.sw_seg.set_color(self.seg_color)

    def _seg_pick(self):
        c = QColorDialog.getColor(QColor(*self.seg_color), self, "选择段颜色")
        if c.isValid():
            self.seg_color = (c.red(), c.green(), c.blue())
            self.sw_seg.set_color(self.seg_color)

    def _seg_read_fields(self):
        n = max(1, self.logical_len)
        start = max(0, min(n - 1, self.sp_seg_start.value()))
        length = max(1, min(n - start, self.sp_seg_len.value()))
        mode = MODE_BY_NAME.get(self.cmb_seg_mode.currentText(), 2)
        r, g, b = self.seg_color
        return {"start": start, "length": length, "mode": mode, "r": r, "g": g, "b": b}

    def seg_add(self):
        if len(self.segs) >= MAX_SEG:
            self.log("分段已达上限 %d" % MAX_SEG)
            return
        self.segs.append(self._seg_read_fields())
        self.seg_apply()

    def seg_update(self):
        row = self.lst_seg.currentRow()
        if row < 0:
            return
        self.segs[row] = self._seg_read_fields()
        self.seg_apply()

    def seg_delete(self):
        row = self.lst_seg.currentRow()
        if row < 0:
            return
        del self.segs[row]
        self.seg_apply()

    def seg_clear(self):
        self.segs = []
        self.seg_apply()

    def seg_apply(self):
        self._seg_refresh_list()
        if self.syncing:
            return
        self.send("PLAY 0")
        self.send("SEGCLR")
        for i, g in enumerate(self.segs):
            self.send("SEG %d %d %d %d %d %d %d" % (
                i, g["start"], g["length"], g["mode"], g["r"], g["g"], g["b"]))
        self.state["segCount"] = len(self.segs)

    # ------------------------------------------------------------ 像素画
    def _pix_click(self, i):
        if not (0 <= i < len(self.pix)):
            return
        self.pix[i] = self.brush
        self.cv_pix.set_pixels(self.pix)
        self.send("PIX %d %d %d %d" % (i, self.brush[0], self.brush[1], self.brush[2]))

    def _set_brush(self, hexcolor):
        self.brush = hex2rgb(hexcolor)
        self.lbl_brush.setText("画笔 %s" % hexcolor)

    def _brush_pick(self):
        c = QColorDialog.getColor(QColor(*self.brush), self, "选择画笔颜色")
        if c.isValid():
            self.brush = (c.red(), c.green(), c.blue())
            self.lbl_brush.setText("画笔 %s" % rgb2hex(self.brush))

    def pix_fill(self):
        self.pix = [self.brush] * self.logical_len
        self.cv_pix.set_pixels(self.pix)
        self.pix_apply()

    def pix_clear(self):
        self.pix = [(0, 0, 0)] * self.logical_len
        self.cv_pix.set_pixels(self.pix)
        self.send("PIXCLR")

    _PIX_CHUNK = 32

    def _send_pix_chunks(self):
        n = len(self.pix)
        for s in range(0, n, self._PIX_CHUNK):
            part = self.pix[s:s + self._PIX_CHUNK]
            flat = " ".join("%d %d %d" % p for p in part)
            self.send("PIXB %d %s" % (s, flat), quiet=(s != 0))

    def pix_apply(self):
        self._send_pix_chunks()
        self.send("PLAY 0")
        self.send("SEGCLR")
        self.segs = []
        self._seg_refresh_list()
        self.state["mode"] = 13
        self._highlight_mode()
        self.send("MODE 13")

    # ------------------------------------------------------------ 编排
    def _step_refresh_list(self):
        self.lst_step.clear()
        for i, t in enumerate(self.steps):
            self.lst_step.addItem("%d.   %s    %d 秒    %s → %s" % (
                i + 1, MODE_NAME_BY_VAL.get(t["mode"], "?"), t["secs"],
                rgb2hex((t["r1"], t["g1"], t["b1"])),
                rgb2hex((t["r2"], t["g2"], t["b2"]))))

    def _step_select(self, row):
        if row < 0 or row >= len(self.steps):
            return
        t = self.steps[row]
        self.cmb_step_mode.setCurrentText(MODE_NAME_BY_VAL.get(t["mode"], "呼吸"))
        self.sp_step_secs.setValue(t["secs"])
        self.step_c1 = (t["r1"], t["g1"], t["b1"])
        self.step_c2 = (t["r2"], t["g2"], t["b2"])
        self.sw_step1.set_color(self.step_c1)
        self.sw_step2.set_color(self.step_c2)

    def _step_pick(self, which):
        cur = self.step_c1 if which == 1 else self.step_c2
        c = QColorDialog.getColor(QColor(*cur), self, "选择颜色")
        if not c.isValid():
            return
        col = (c.red(), c.green(), c.blue())
        if which == 1:
            self.step_c1 = col
            self.sw_step1.set_color(col)
        else:
            self.step_c2 = col
            self.sw_step2.set_color(col)

    def _step_read_fields(self):
        secs = max(1, min(3600, self.sp_step_secs.value()))
        mode = MODE_BY_NAME.get(self.cmb_step_mode.currentText(), 2)
        return {"mode": mode, "secs": secs,
                "r1": self.step_c1[0], "g1": self.step_c1[1], "b1": self.step_c1[2],
                "r2": self.step_c2[0], "g2": self.step_c2[1], "b2": self.step_c2[2]}

    def step_add(self):
        if len(self.steps) >= MAX_STEP:
            self.log("步骤已达上限 %d" % MAX_STEP)
            return
        self.steps.append(self._step_read_fields())
        self.step_apply()

    def step_update(self):
        row = self.lst_step.currentRow()
        if row < 0:
            return
        self.steps[row] = self._step_read_fields()
        self.step_apply()

    def step_delete(self):
        row = self.lst_step.currentRow()
        if row < 0:
            return
        del self.steps[row]
        self.step_apply()

    def step_clear(self):
        self.steps = []
        self.step_apply()

    def step_apply(self):
        self._step_refresh_list()
        if self.syncing:
            return
        self.send("STEPCLR")
        for i, t in enumerate(self.steps):
            self.send("STEP %d %d %d %d %d %d %d %d %d" % (
                i, t["mode"], t["secs"], t["r1"], t["g1"], t["b1"],
                t["r2"], t["g2"], t["b2"]))
        self.state["stepCount"] = len(self.steps)

    def play_start(self):
        if not self.steps:
            self.log("请先添加编排步骤")
            return
        self.state["playing"] = 1
        self.send("PLAY 1")

    def play_stop(self):
        self.state["playing"] = 0
        self.send("PLAY 0")

    def play_next(self):
        self.send("PLAYNEXT")

    # ------------------------------------------------------------ 音乐
    def refresh_devices(self):
        self.cmb_dev.clear()
        if not AUDIO_OK:
            self.log("未安装音频依赖，无法枚举音频设备")
            return
        n = 0
        if HAVE_SC:
            try:
                for m in sc.all_microphones(include_loopback=True):
                    tag = "系统声音" if m.isloopback else "麦克风"
                    self.cmb_dev.addItem("[%s] %s" % (tag, m.name),
                                         ("sc", m.name, bool(m.isloopback)))
                    n += 1
            except Exception as e:
                self.log("枚举音频设备失败: %s" % e)
        elif HAVE_SD:
            try:
                for i, d in enumerate(sd.query_devices()):
                    if d.get("max_input_channels", 0) > 0:
                        self.cmb_dev.addItem("[麦克风] %s" % d.get("name", "?"),
                                             ("sd", i, False))
                        n += 1
            except Exception as e:
                self.log("枚举设备失败: %s" % e)
        idx = self._default_source_index()
        if idx >= 0:
            self.cmb_dev.setCurrentIndex(idx)
        self.log("发现音频来源 %d 个" % self.cmb_dev.count())

    def _default_source_index(self):
        """优先选中当前默认扬声器对应的系统声音（回环）来源。"""
        speaker = None
        if HAVE_SC:
            try:
                speaker = sc.default_speaker().name
            except Exception:
                speaker = None
        for i in range(self.cmb_dev.count()):
            d = self.cmb_dev.itemData(i)
            if d and d[0] == "sc" and d[2] and speaker and d[1] == speaker:
                return i
        for i in range(self.cmb_dev.count()):
            d = self.cmb_dev.itemData(i)
            if d and d[0] == "sc" and d[2]:
                return i
        return 0 if self.cmb_dev.count() else -1

    def toggle_music(self):
        if self.music_run:
            self.music_stop()
        else:
            self.music_start()

    def set_idle(self):
        val = self.cmb_idle.currentData()
        if val is None or self.syncing:
            return
        self.state["idle"] = val
        self.send("IDLE %d" % val)

    def music_start(self, auto=False):
        if not AUDIO_OK:
            if auto:
                self.log("音乐律动未自动启动：缺少 soundcard/numpy 依赖")
            else:
                QMessageBox.warning(self, "缺少依赖",
                                    "音乐律动需要安装：pip install soundcard numpy")
            return
        if self.music_run:
            return
        data = self.cmb_dev.currentData()
        if not data:
            self.log("音乐律动未启动：未找到可用音频来源")
            return
        self._cap_kind = "loop" if data[2] else data[0]
        self._cap_dev = data[1]
        self._cap_stop.clear()
        self.mic_level = 0
        self.level_l = 0
        self.level_r = 0
        self._cap_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._cap_thread.start()
        self.music_run = True
        self.btn_music.setText("停止律动")
        rmode = self.cmb_rmode.currentData()
        self.state["mode"] = rmode
        self._highlight_mode()
        self.send("IDLE %d" % self.state["idle"])
        self.send("MODE %d" % rmode)
        self._music_timer.start()
        self.log("音乐律动已启动（%s，来源：%s）"
                 % (self.cmb_rmode.currentText(), self.cmb_dev.currentText()))

    def music_stop(self):
        if not self.music_run and self._cap_thread is None:
            return
        self.music_run = False
        self._music_timer.stop()
        self._cap_stop.set()
        th = self._cap_thread
        self._cap_thread = None
        if th is not None:
            th.join(timeout=1.5)
        self.mic_level = 0
        self.level_l = 0
        self.level_r = 0
        self.meter.set_level(0)
        self.stereo_meter.set_levels(0, 0)
        self.btn_music.setText("启动律动")
        self.log("音乐律动已停止")

    def _capture_loop(self):
        """后台采集线程：阻塞式读取音频块，换算成电平。"""
        try:
            if self._cap_kind == "loop":
                mic = sc.get_microphone(self._cap_dev, include_loopback=True)
                with mic.recorder(samplerate=48000, channels=None) as rec:
                    while not self._cap_stop.is_set():
                        self._push_level(rec.record(numframes=1024))
            elif self._cap_kind == "sc":
                mic = sc.get_microphone(self._cap_dev)
                with mic.recorder(samplerate=48000, channels=None) as rec:
                    while not self._cap_stop.is_set():
                        self._push_level(rec.record(numframes=1024))
            else:
                with sd.InputStream(device=self._cap_dev, channels=1) as stream:
                    while not self._cap_stop.is_set():
                        chunk, _ = stream.read(1024)
                        self._push_level(chunk)
        except Exception as e:
            if not self._cap_stop.is_set():
                self.capError.emit("音频采集失败: %s" % e)

    def _push_level(self, data):
        try:
            arr = np.asarray(data, dtype="float32")
            if arr.ndim > 1 and arr.shape[1] >= 2:
                rl = float(np.sqrt(np.mean(np.square(arr[:, 0]))))
                rr = float(np.sqrt(np.mean(np.square(arr[:, 1:]))))
            else:
                rl = rr = float(np.sqrt(np.mean(np.square(arr))))
        except Exception:
            rl = rr = 0.0
        lv_l = int(min(255, rl * 700))
        lv_r = int(min(255, rr * 700))
        self.level_l = self._smooth(self.level_l, lv_l)
        self.level_r = self._smooth(self.level_r, lv_r)
        self.mic_level = self._smooth(self.mic_level, max(lv_l, lv_r))

    @staticmethod
    def _smooth(prev, new):
        # 快攻慢放，灯效过渡更顺滑
        return new if new >= prev else int(prev * 0.7 + new * 0.3)

    def _music_tick(self):
        if not self.music_run:
            return
        self.send("LEVEL %d %d %d" % (self.mic_level, self.level_l, self.level_r),
                  quiet=True)
        self.meter.set_level(self.mic_level)
        self.stereo_meter.set_levels(self.level_l, self.level_r)

    # ------------------------------------------------------------ 预设
    def load_presets(self):
        try:
            if os.path.exists(PRESET_FILE):
                with open(PRESET_FILE, "r", encoding="utf-8") as f:
                    self.presets = json.load(f)
        except Exception as e:
            self.log("读取预设失败: %s" % e)
            self.presets = {}
        self._refresh_preset_combo()

    def _refresh_preset_combo(self):
        self.cmb_preset.clear()
        self.cmb_preset.addItems(sorted(self.presets.keys()))

    def _save_presets_file(self):
        try:
            with open(PRESET_FILE, "w", encoding="utf-8") as f:
                json.dump(self.presets, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.log("保存预设文件失败: %s" % e)

    def _snapshot(self):
        return {
            "state": dict(self.state),
            "segs": [dict(g) for g in self.segs],
            "steps": [dict(t) for t in self.steps],
            "pix": [list(p) for p in self.pix],
            "strips": {
                "numStrips": self.strip_cfg["numStrips"],
                "merge": self.strip_cfg["merge"],
                "strips": [dict(d) for d in self.strip_cfg["strips"]],
            },
        }

    def _cmds_from_snapshot(self, snap):
        s = snap["state"]
        out = []
        st = snap.get("strips")
        if st:
            out.append("STRIPS %d %d" % (st["numStrips"], st["merge"]))
            for i in range(st["numStrips"]):
                d = st["strips"][i]
                out.append("STRIPSET %d %d %d %d" % (
                    i, d["pin"], d["count"], d["enabled"]))
        out += ["BRIGHT %d" % s["bright"], "SPEED %d" % s["speed"],
               "SYNC %d" % s["sync"],
               "COLOR %d %d %d" % (s["r1"], s["g1"], s["b1"]),
               "COLOR2 %d %d %d" % (s["r2"], s["g2"], s["b2"]),
               "HUE %d" % s["hueSpan"], "DIR %d" % s["dir"], "TAIL %d" % s["tail"],
               "SPACING %d" % s["spacing"], "MIRROR %d" % s["mirror"],
               "RANDOM %d" % s["random"], "GAMMA %d" % s["gamma"],
               "GAIN %d" % s["gain"], "IDLE %d" % s.get("idle", 2), "PLAY 0"]
        out.append("SEGCLR")
        for i, g in enumerate(snap["segs"]):
            out.append("SEG %d %d %d %d %d %d %d" % (i, g["start"], g["length"],
                                                     g["mode"], g["r"], g["g"], g["b"]))
        out.append("STEPCLR")
        for i, t in enumerate(snap["steps"]):
            out.append("STEP %d %d %d %d %d %d %d %d %d" % (
                i, t["mode"], t["secs"], t["r1"], t["g1"], t["b1"],
                t["r2"], t["g2"], t["b2"]))
        pix = snap["pix"]
        for k in range(0, len(pix), self._PIX_CHUNK):
            part = pix[k:k + self._PIX_CHUNK]
            out.append("PIXB %d %s" % (
                k, " ".join("%d %d %d" % tuple(p) for p in part)))
        out.append("MODE %d" % s["mode"])
        if s.get("playing") and snap["steps"]:
            out.append("PLAY 1")
        return out

    def preset_save(self):
        name = self.cmb_preset.currentText().strip()
        if not name:
            name = "预设%d" % (len(self.presets) + 1)
        self.presets[name] = self._snapshot()
        self._save_presets_file()
        self._refresh_preset_combo()
        self.cmb_preset.setCurrentText(name)
        self.log("已保存预设：%s" % name)

    def preset_delete(self):
        name = self.cmb_preset.currentText().strip()
        if name in self.presets:
            del self.presets[name]
            self._save_presets_file()
            self._refresh_preset_combo()
            self.log("已删除预设：%s" % name)

    def preset_load(self):
        name = self.cmb_preset.currentText().strip()
        snap = self.presets.get(name)
        if not snap:
            self.log("请选择要加载的预设")
            return
        self.syncing = True
        try:
            self.state.update(snap["state"])
            st = snap.get("strips")
            if st:
                self.strip_cfg = {
                    "numStrips": st["numStrips"],
                    "merge": st["merge"],
                    "strips": [dict(d) for d in st["strips"]],
                }
                self._apply_logical_len(self._logical_from_cfg(self.strip_cfg))
            self.segs = [dict(g) for g in snap["segs"]]
            self.steps = [dict(t) for t in snap["steps"]]
            self.pix = [tuple(p) for p in snap["pix"]]
            self._refresh_all()
        finally:
            self.syncing = False
        for c in self._cmds_from_snapshot(snap):
            self.send(c)
        self.log("已加载预设：%s" % name)

    def preset_import(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "导入预设", "", "JSON (*.json);;所有文件 (*)")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                self.presets.update(data)
                self._save_presets_file()
                self._refresh_preset_combo()
                self.log("已导入 %d 个预设" % len(data))
        except Exception as e:
            self.log("导入失败: %s" % e)

    def preset_export(self):
        if not self.presets:
            self.log("当前没有预设可导出")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "导出预设", "presets.json", "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.presets, f, ensure_ascii=False, indent=2)
            self.log("已导出预设：%s" % path)
        except Exception as e:
            self.log("导出失败: %s" % e)

    # ------------------------------------------------------------ 托盘 / 启动
    def _tray_icon(self):
        """优先加载 icon.ico（高清多尺寸）；缺失时用代码绘制（渐变圆 + 白色灯点）。"""
        if os.path.exists(ICON_FILE):
            ic = QIcon(ICON_FILE)
            if not ic.isNull():
                return ic
        pm = QPixmap(32, 32)
        pm.fill(Qt.transparent)
        p = QPainter(pm)
        p.setRenderHint(QPainter.Antialiasing)
        grad = QLinearGradient(0, 0, 32, 32)
        grad.setColorAt(0, QColor("#667eea"))
        grad.setColorAt(1, QColor("#8b5cf6"))
        p.setPen(Qt.NoPen)
        p.setBrush(grad)
        p.drawEllipse(2, 2, 28, 28)
        p.setBrush(QColor("#ffffff"))
        p.drawEllipse(13, 13, 6, 6)
        p.end()
        return QIcon(pm)

    def _build_tray(self):
        if not QSystemTrayIcon.isSystemTrayAvailable():
            self.log("系统托盘不可用，改为普通窗口模式")
            self._show_window()
            return
        self.tray = QSystemTrayIcon(self._tray_icon(), self)
        self.tray.setToolTip("ESP32-C3 神光同步工作台")
        menu = QMenu()
        act_show = QAction("显示控制台", self)
        act_show.triggered.connect(self._show_window)
        self.act_conn = QAction("连接串口", self)
        self.act_conn.triggered.connect(self.toggle_conn)
        act_quit = QAction("退出", self)
        act_quit.triggered.connect(self._quit_app)
        menu.addAction(act_show)
        menu.addAction(self.act_conn)
        menu.addSeparator()
        menu.addAction(act_quit)
        self.tray.setContextMenu(menu)
        self.tray.activated.connect(self._on_tray_activated)
        self.tray.messageClicked.connect(self._show_window)
        self.tray.show()
        self.tray_ok = True

    def _on_tray_activated(self, reason):
        if reason in (QSystemTrayIcon.Trigger, QSystemTrayIcon.DoubleClick):
            self._show_window()

    def _show_window(self):
        self.showNormal()               # 最小化状态也能恢复
        self.raise_()
        self.activateWindow()

    def _quit_app(self):
        self._really_quit = True
        self.close()

    def _startup_autoconnect(self):
        """程序默认在托盘运行：不弹窗，自动连接上次记住的默认端口。"""
        if self.tray_ok:
            self.tray.showMessage(
                "已在后台运行",
                "点击托盘图标打开控制台，正在连接默认串口…",
                QSystemTrayIcon.Information, 2500)
        port = str(self.settings.value("port", "") or "")
        if port:
            for i in range(self.cmb_port.count()):
                if self.cmb_port.itemText(i).split()[0] == port:
                    self.cmb_port.setCurrentIndex(i)
                    break
        if self.cmb_port.count() == 0:
            self.log("未发现串口，已打开控制台")
            self._show_window()
            return
        self.log("默认端口：%s，正在自动连接" % (port or self._cur_port()))
        self.connect()

    # ------------------------------------------------------------ 日志 / 收尾
    def log(self, text):
        self.log_box.appendPlainText(text)
        sb = self.log_box.verticalScrollBar()
        sb.setValue(sb.maximum())

    def closeEvent(self, event):
        # 关闭窗口默认收进托盘继续在后台跑；只有托盘菜单「退出」才真正结束
        if self.tray_ok and not self._really_quit:
            event.ignore()
            self.hide()
            if not self._tray_tip_shown:
                self._tray_tip_shown = True
                self.tray.showMessage(
                    "仍在后台运行",
                    "程序已收进托盘，点击托盘图标可重新打开。",
                    QSystemTrayIcon.Information, 2500)
            return
        if self.music_run:
            self.music_stop()
        worker, th = self.worker, self.thread
        self.worker = None
        self.thread = None
        if worker:
            worker.stop()
        if th is not None:
            th.quit()
            if not th.wait(1500):
                th.terminate()
                th.wait(500)
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setApplicationName("ESP32-C3 神光同步工作台")
    app.setOrganizationName("ESP32C3-ARGB")
    app.setStyleSheet(QSS)
    app.setQuitOnLastWindowClosed(False)     # 关窗不退出，常驻托盘
    win = MainWindow()
    # 默认在托盘运行：不主动弹窗，点击托盘图标即可呼出控制台
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
