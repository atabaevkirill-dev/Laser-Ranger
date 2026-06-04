"""
3km Eye-Safe Laser Rangefinder — Desktop Application
• Serial / TCP-IP connection to rangefinder
• Single Shot + Start/Stop Continuous ranging
• IP camera (RTSP/OpenCV) with military reticle overlay
  - 5 target types (infantry, vehicle, aircraft, building, uav)
  - 5 reticle styles (crosshair, mil-dot, BDC, circle, tactical)
  - 6 colors, range scale, HUD, scale/brightness/opacity controls
"""

import sys
import serial
import threading
import time
import socket
import configparser
import math
import logging

import cv2
import numpy as np

# Configure logging so protocol_handler/debug messages actually appear
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(name)s %(levelname)s: %(message)s',
    datefmt='%H:%M:%S',
)
logger = logging.getLogger(__name__)

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QTextEdit, QMessageBox,
    QGroupBox, QSpinBox, QGridLayout, QProgressBar,
    QMenuBar, QAction, QStatusBar, QLineEdit, QSplitter,
    QSlider, QCheckBox,
)
from PyQt5.QtCore  import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui   import QFont, QColor, QPalette, QImage, QPixmap

from protocol_handler     import ProtocolHandler
from tcp_protocol_handler import TcpProtocolHandler
from protocol_base        import CMD_CONTINUE_RANGING, CMD_RANGING_ABNORMAL, decode_ranging_response
from overlay_renderer     import OverlayState, render as overlay_render, COLORS

try:
    from onvif_camera_control import OnvifCameraController, PelcoDController, UniversalCameraController
    ONVIF_AVAILABLE = True
except ImportError:
    ONVIF_AVAILABLE = False
    UniversalCameraController = None
    print("ONVIF library not available. Install 'onvif_zeep' to enable ONVIF camera control.")

# Import legacy pan-tilt controller (optional)
try:
    from pan_tilt_controller import PanTiltController
    PAN_TILT_AVAILABLE = True
except ImportError:
    PAN_TILT_AVAILABLE = False
    print("Pan-tilt controller not available.")

# Создаем специальный QLabel, который может обрабатывать колесо мыши
class CameraLabel(QLabel):
    """Специальная метка для камеры, которая обрабатывает события колеса мыши для зума"""
    mouse_wheel_event = pyqtSignal(int)  # Сигнал для передачи направления вращения колеса
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        
    def wheelEvent(self, event):
        """Обработка события колеса мыши"""
        delta = event.angleDelta().y()
        if delta > 0:
            self.mouse_wheel_event.emit(1)  # Вращение вперед (увеличение)
        else:
            self.mouse_wheel_event.emit(-1)  # Вращение назад (уменьшение)
        event.accept()

# ═══════════════════════════════════════════════════════════════════════════
#  Стили
# ═══════════════════════════════════════════════════════════════════════════
DARK_STYLE = """
QMainWindow,QWidget,QGroupBox{background-color:#2b2b2b;color:#fff}
QLabel{color:#fff;background-color:transparent}
QPushButton{background:#3d3d3d;border:1px solid #5a5a5a;padding:5px 10px;border-radius:3px;color:#fff}
QPushButton:hover{background:#4d4d4d}
QPushButton:pressed{background:#2a2a2a}
QPushButton:disabled{background:#222;color:#555}
QPushButton#btn_single{background:#163a6b;border:1px solid #2a6acc;font-weight:bold}
QPushButton#btn_single:hover{background:#1d4d8a}
QPushButton#btn_cont{background:#16502a;border:1px solid #25a04a;font-weight:bold}
QPushButton#btn_cont:hover{background:#1a6035}
QPushButton#btn_cont_stop{background:#6b1616;border:1px solid #cc2a2a;font-weight:bold}
QComboBox,QSpinBox,QLineEdit{background:#222;border:1px solid #5a5a5a;padding:3px;color:#fff}
QTextEdit{background:#1a1a1a;border:1px solid #5a5a5a;color:#eee}
QMenuBar{background:#2b2b2b;color:#fff}
QMenuBar::item:selected{background:#3d3d3d}
QStatusBar{background:#2b2b2b;color:#fff}
QProgressBar{border:1px solid #5a5a5a;text-align:center;background:#1a1a1a}
QProgressBar::chunk{background:#2a7acc}
QGroupBox{font-weight:bold;border:1px solid #5a5a5a;border-radius:5px;
           margin-top:1ex;padding-top:10px;color:#ccc;background:#2b2b2b}
QSlider::groove:horizontal{height:4px;background:#444;border-radius:2px}
QSlider::handle:horizontal{width:14px;height:14px;background:#5a9fd4;border-radius:7px;margin:-5px 0}
QCheckBox{color:#ccc}
QCheckBox::indicator:checked{background:#2a7acc;border:1px solid #5aacff}
"""

DIST_OK   = "QLabel{background:#1a4a28;padding:8px;font-size:20px;font-weight:bold;border-radius:4px;color:#88ff88}"
DIST_ERR  = "QLabel{background:#4a1a1a;padding:8px;font-size:20px;font-weight:bold;border-radius:4px;color:#ff8888}"
DIST_IDLE = "QLabel{background:#2b2b2b;padding:8px;font-size:20px;font-weight:bold;border-radius:4px;color:#888}"

# ═══════════════════════════════════════════════════════════════════════════
#  Camera thread
# ═══════════════════════════════════════════════════════════════════════════
class CameraThread(QThread):
    frame_ready = pyqtSignal(np.ndarray)
    error       = pyqtSignal(str)

    def __init__(self, url: str, overlay: OverlayState, config):
        super().__init__()
        self.url     = url
        self.overlay = overlay
        self.config = config
        self._active = False

    def run(self):
        self._active = True
        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        # Получаем параметры из конфигурации для уменьшения задержки
        buffer_size = self.config.getint('CAMERA', 'buffer_size', fallback=1)
        frame_width = self.config.getint('CAMERA', 'frame_width', fallback=640)
        frame_height = self.config.getint('CAMERA', 'frame_height', fallback=480)
        fps = self.config.getint('CAMERA', 'fps', fallback=30)
        
        # Устанавливаем минимальный размер буфера для уменьшения задержки
        cap.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)
        # Дополнительные параметры для уменьшения задержки
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        # Отключаем автоподстройку яркости, контраста и насыщенности для уменьшения задержки
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        
        if not cap.isOpened():
            self.error.emit(f"Cannot open: {self.url}")
            return
        while self._active:
            ret, frame = cap.read()
            if not ret:
                self.error.emit("Stream lost")
                break
            # Выполняем отрисовку оверлея только когда действительно нужно
            frame = overlay_render(frame, self.overlay)
            self.frame_ready.emit(frame)
            # Уменьшаем задержку между кадрами для более быстрого обновления
            self.msleep(10)  # Уменьшено с 33 до 10 мс для более частого обновления
        cap.release()

    def stop(self):
        self._active = False
        self.wait(3000)

# ═══════════════════════════════════════════════════════════════════════════
#  Main window
# ═══════════════════════════════════════════════════════════════════════════
class LaserRangefinderApp(QMainWindow):
    update_signal       = pyqtSignal(str, str)
    log_signal          = pyqtSignal(str)
    update_multi_signal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self._apply_dark_theme()

        self.serial_conn    = None
        self.tcp_socket     = None
        self.protocol       = None
        self.is_connected   = False
        self.is_ranging     = False
        self._is_ranging_event = threading.Event()   # Thread-safe ranging signal
        self._multi_targets_lock = threading.Lock()  # Protects multi_targets dict
        self.ranging_thread = None
        self.connection_type = "serial"
        self.multi_targets  = {}

        self.overlay        = OverlayState()
        self.camera_thread  = None
        self._last_frame_rgb = None  # Hold reference to prevent QImage dangling pointer

        self.config = configparser.ConfigParser()
        self.config.read('config.ini', encoding='utf-8')

        # Camera controller (pan-tilt movement via Pelco-D)
        self.camera_controller = None
        # Zoom controller (camera lens zoom via ONVIF)
        self.zoom_controller = None
        self.use_onvif = self.config.getboolean('CAMERA', 'enable_onvif', fallback=False)
        self.use_pelco_d = self.config.getboolean('CAMERA', 'use_pelco_d', fallback=False)
        
        # Инициализация контроллера поворотки (новый функционал)
        self.pan_tilt_controller = None
        self.use_pan_tilt = self.config.getboolean('PAN_TILT', 'use_pan_tilt', fallback=False)
        if self.use_pan_tilt and PAN_TILT_AVAILABLE:
            pan_tilt_host = self.config.get('PAN_TILT', 'host', fallback='192.168.1.115')
            pan_tilt_port = self.config.getint('PAN_TILT', 'port', fallback=9760)
            self.pan_tilt_controller = PanTiltController(pan_tilt_host, pan_tilt_port)
            if self.pan_tilt_controller.connect():
                print(f"Successfully connected to pan-tilt controller at {pan_tilt_host}:{pan_tilt_port}")
            else:
                print(f"Failed to connect to pan-tilt controller at {pan_tilt_host}:{pan_tilt_port}")
                self.pan_tilt_controller = None  # Устанавливаем в None, если соединение не удалось

        
        # Initialize ONVIF zoom controller (separate from pan-tilt)
        # Camera lens zoom is always via ONVIF to the camera itself
        zoom_method = self.config.get('CAMERA', 'zoom_method', fallback='onvif')
        if zoom_method == 'onvif' and ONVIF_AVAILABLE:
            onvif_ip = self.config.get('CAMERA', 'onvif_ip', fallback='192.168.1.68')
            onvif_user = self.config.get('CAMERA', 'onvif_username', fallback='admin')
            onvif_pass = self.config.get('CAMERA', 'onvif_password', fallback='12qwaszx')
            onvif_port = self.config.getint('CAMERA', 'onvif_port', fallback=80)
            self.zoom_controller = OnvifCameraController(onvif_ip, onvif_user, onvif_pass, onvif_port)
            if self.zoom_controller.connect():
                print(f"ONVIF zoom controller connected to {onvif_ip}")
            else:
                print(f"Failed to connect ONVIF zoom controller to {onvif_ip}")
                self.zoom_controller = None
        
        # Initialize pan-tilt camera controller (Pelco-D for movement)
        camera_control_method = self.config.get('CAMERA', 'camera_control_method', fallback='onvif')
        if camera_control_method == 'onvif' and ONVIF_AVAILABLE:
            onvif_ip = self.config.get('CAMERA', 'onvif_ip', fallback='192.168.1.68')
            onvif_user = self.config.get('CAMERA', 'onvif_username', fallback='admin')
            onvif_pass = self.config.get('CAMERA', 'onvif_password', fallback='12qwaszx')
            onvif_port = self.config.getint('CAMERA', 'onvif_port', fallback=80)
            self.camera_controller = UniversalCameraController(
                control_method='onvif',
                ip=onvif_ip,
                username=onvif_user,
                password=onvif_pass,
                port=onvif_port
            )
        elif camera_control_method == 'pelco_d' and ONVIF_AVAILABLE:
            pelco_d_ip = self.config.get('CAMERA', 'pelco_d_ip', fallback='192.168.1.120:80')
            self.camera_controller = UniversalCameraController(
                control_method='pelco_d',
                ip=pelco_d_ip
            )
        
        # Попытка подключения при инициализации
        if hasattr(self, 'camera_controller') and self.camera_controller:
            if self.camera_controller.connect():
                print(f"Successfully connected to camera via {camera_control_method} at {self.camera_controller.ip}")
            else:
                print(f"Failed to connect to camera via {camera_control_method}")
        
        # Connection health state
        self.connection_healthy = True
        self.continuous_measurements_active = False
        
        self._build_ui()
        self._update_connection_status()

        self.update_signal.connect(self._on_update_display)
        self.log_signal.connect(self._on_log)
        self.update_multi_signal.connect(self._on_multi_targets)

    def _apply_dark_theme(self):
        pal = QPalette()
        pal.setColor(QPalette.Window,     QColor(43, 43, 43))
        pal.setColor(QPalette.WindowText, Qt.white)
        pal.setColor(QPalette.Base,       QColor(26, 26, 26))
        pal.setColor(QPalette.Text,       Qt.white)
        pal.setColor(QPalette.Button,     QColor(43, 43, 43))
        pal.setColor(QPalette.ButtonText, Qt.white)
        pal.setColor(QPalette.Highlight,  QColor(42, 122, 204))
        pal.setColor(QPalette.HighlightedText, Qt.black)
        pal.setColor(QPalette.Disabled, QPalette.Text,       QColor(100,100,100))
        pal.setColor(QPalette.Disabled, QPalette.ButtonText, QColor(100,100,100))
        self.setPalette(pal)
        self.setStyleSheet(DARK_STYLE)

    # ─────────────────────────────────────────────────────────────────
    #  BUILD UI
    # ─────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self.setWindowTitle("3km Eye-Safe Laser Rangefinder")
        self.setGeometry(
            self.config.getint('UI_SETTINGS','window_x',fallback=60),
            self.config.getint('UI_SETTINGS','window_y',fallback=60),
            self.config.getint('UI_SETTINGS','window_width',fallback=1400),
            self.config.getint('UI_SETTINGS','window_height',fallback=860),
        )

        mb = QMenuBar(); self.setMenuBar(mb)
        fm = mb.addMenu('File'); fm.addAction(QAction('Exit', self, triggered=self.close))
        hm = mb.addMenu('Help'); hm.addAction(QAction('About', self, triggered=self._show_about))

        self.status_bar = QStatusBar(); self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Disconnected")

        # Горизонтальный сплиттер: [левая панель | камера]
        splitter = QSplitter(Qt.Horizontal)
        self.setCentralWidget(splitter)

        # ── ЛЕВАЯ ПАНЕЛЬ ──
        left = QWidget()
        ll   = QVBoxLayout(left); ll.setSpacing(6)
        title = QLabel("3km Eye-Safe Laser Rangefinder")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 13, QFont.Bold))
        title.setStyleSheet("background:#1a1a1a;color:#fff;padding:7px;border-radius:4px")
        ll.addWidget(title)
        ll.addWidget(self._build_connection_group())
        ll.addWidget(self._build_ranging_group())
        ll.addWidget(self._build_results_group())
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(False); self.progress_bar.setVisible(False)
        ll.addWidget(self.progress_bar)
        ll.addWidget(self._build_info_group())
        ll.addWidget(self._build_camera_controls_group())  # Добавляем элементы управления камерой
        ll.addStretch()

        # ── ПРАВАЯ ПАНЕЛЬ (камера + прицел) ──
        right = QWidget()
        rl    = QVBoxLayout(right); rl.setSpacing(6)
        rl.addWidget(self._build_camera_group())
        rl.addWidget(self._build_overlay_group())

        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([400, 1000])

        self.timer = QTimer(); self.timer.timeout.connect(self._tick_ui); self.timer.start(500)

    def _build_camera_controls_group(self):
        """Создание группы элементов управления камерой."""
        grp = QGroupBox("Camera & Pan-Tilt Controls"); lay = QVBoxLayout(grp)
        
        # Создаем кнопки управления камерой и повороткой через универсальный контроллер
        if hasattr(self, 'camera_controller') and self.camera_controller:
            camera_control_layout = QGridLayout()
            
            # Диагональные кнопки для универсального контроллера
            self.diag_up_left_btn = QPushButton("↖")
            self.diag_up_left_btn.pressed.connect(lambda: self._camera_move_diagonal(-1, 1))
            self.diag_up_left_btn.released.connect(self._camera_stop)  # Останавливает обе оси
            camera_control_layout.addWidget(self.diag_up_left_btn, 0, 0)
            
            self.up_btn = QPushButton("↑")
            self.up_btn.pressed.connect(lambda: self._camera_move_tilt(1))
            self.up_btn.released.connect(self._camera_stop_tilt)  # Останавливает только tilt
            camera_control_layout.addWidget(self.up_btn, 0, 1)
            
            self.diag_up_right_btn = QPushButton("↗")
            self.diag_up_right_btn.pressed.connect(lambda: self._camera_move_diagonal(1, 1))
            self.diag_up_right_btn.released.connect(self._camera_stop)  # Останавливает обе оси
            camera_control_layout.addWidget(self.diag_up_right_btn, 0, 2)
            
            self.left_btn = QPushButton("←")
            self.left_btn.pressed.connect(lambda: self._camera_move_pan(-1))
            self.left_btn.released.connect(self._camera_stop_pan)  # Останавливает только pan
            camera_control_layout.addWidget(self.left_btn, 1, 0)
            
            # Меняем местами кнопку "Стоп" и "Домой"
            self.stop_btn = QPushButton("⏹ Stop")
            self.stop_btn.clicked.connect(self._camera_stop)  # Останавливает обе оси
            camera_control_layout.addWidget(self.stop_btn, 1, 1)
            
            self.right_btn = QPushButton("→")
            self.right_btn.pressed.connect(lambda: self._camera_move_pan(1))
            self.right_btn.released.connect(self._camera_stop_pan)  # Останавливает только pan
            camera_control_layout.addWidget(self.right_btn, 1, 2)
            
            self.diag_down_left_btn = QPushButton("↙")
            self.diag_down_left_btn.pressed.connect(lambda: self._camera_move_diagonal(-1, -1))
            self.diag_down_left_btn.released.connect(self._camera_stop)  # Останавливает обе оси
            camera_control_layout.addWidget(self.diag_down_left_btn, 2, 0)
            
            self.down_btn = QPushButton("↓")
            self.down_btn.pressed.connect(lambda: self._camera_move_tilt(-1))
            self.down_btn.released.connect(self._camera_stop_tilt)  # Останавливает только tilt
            camera_control_layout.addWidget(self.down_btn, 2, 1)
            
            self.diag_down_right_btn = QPushButton("↘")
            self.diag_down_right_btn.pressed.connect(lambda: self._camera_move_diagonal(1, -1))
            self.diag_down_right_btn.released.connect(self._camera_stop)  # Останавливает обе оси
            camera_control_layout.addWidget(self.diag_down_right_btn, 2, 2)
            
            # Кнопка "Домой" теперь на месте кнопки "Стоп"
            self.home_btn = QPushButton("🏠 Home")
            self.home_btn.clicked.connect(self._camera_go_home)
            camera_control_layout.addWidget(self.home_btn, 3, 0, 1, 3)
            
            # Кнопка самодиагностики
            self.diag_btn = QPushButton("🔧 Self-Test")
            self.diag_btn.clicked.connect(self._camera_start_self_test)
            camera_control_layout.addWidget(self.diag_btn, 4, 0, 1, 3)
            
            # Добавляем метку для позиций
            self.camera_pos_label = QLabel("Pan: --°, Tilt: --°")
            camera_control_layout.addWidget(self.camera_pos_label, 5, 0, 1, 3)
            
            lay.addLayout(camera_control_layout)
        
        # Добавляем слайдер для регулировки скорости
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Movement Speed:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 10)
        self.speed_slider.setValue(5)
        self.speed_label = QLabel("5")
        self.speed_slider.valueChanged.connect(lambda v: self.speed_label.setText(str(v)))
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_label)
        lay.addLayout(speed_layout)
        
        # Активируем кнопки универсального контроллера
        if hasattr(self, 'camera_controller') and self.camera_controller:
            if hasattr(self, 'diag_up_left_btn'):  # Проверяем, были ли созданы кнопки
                for btn in [self.diag_up_left_btn, self.up_btn, 
                           self.diag_up_right_btn, self.left_btn, 
                           self.stop_btn, self.right_btn, 
                           self.diag_down_left_btn, self.down_btn, 
                           self.diag_down_right_btn, self.home_btn,
                           self.diag_btn]:
                    btn.setEnabled(True)
        
        return grp

    # ── Connection ────────────────────────────────────────────────────
    def _build_connection_group(self):
        grp = QGroupBox("Connection"); lay = QGridLayout(grp)

        lay.addWidget(QLabel("Type:"), 0, 0)
        self.conn_type_combo = QComboBox()
        self.conn_type_combo.addItems(["Serial","TCP/IP"])
        self.conn_type_combo.currentTextChanged.connect(self._on_conn_type_changed)
        lay.addWidget(self.conn_type_combo, 0, 1)

        self.serial_port_widget = QWidget()
        sp = QGridLayout(self.serial_port_widget); sp.setContentsMargins(0,0,0,0)
        sp.addWidget(QLabel("Port:"), 0, 0)
        self.port_combo = QComboBox(); sp.addWidget(self.port_combo, 0, 1)
        rb = QPushButton("↻"); rb.setFixedWidth(28); rb.clicked.connect(self._refresh_ports)
        sp.addWidget(rb, 0, 2)
        self._refresh_ports()
        lay.addWidget(self.serial_port_widget, 1, 0, 1, 2)

        self.baud_widget = QWidget()
        bl = QGridLayout(self.baud_widget); bl.setContentsMargins(0,0,0,0)
        bl.addWidget(QLabel("Baud:"), 0, 0)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600","57600","115200"]); self.baud_combo.setCurrentText("115200")
        bl.addWidget(self.baud_combo, 0, 1)
        lay.addWidget(self.baud_widget, 2, 0, 1, 2)

        self.tcp_fields_widget = QWidget()
        tl = QGridLayout(self.tcp_fields_widget); tl.setContentsMargins(0,0,0,0)
        tl.addWidget(QLabel("IP:"), 0, 0)
        self.tcp_ip_input = QLineEdit(self.config.get('LASER_RANGING','tcp_ip_address',fallback='192.168.1.7'))
        tl.addWidget(self.tcp_ip_input, 0, 1)
        tl.addWidget(QLabel("Port:"), 1, 0)
        self.tcp_port_input = QLineEdit(self.config.get('LASER_RANGING','tcp_port',fallback='20108'))
        tl.addWidget(self.tcp_port_input, 1, 1)
        lay.addWidget(self.tcp_fields_widget, 3, 0, 1, 2)

        self.conn_btn = QPushButton("Connect"); self.conn_btn.clicked.connect(self._toggle_connection)
        self.conn_btn.setMinimumHeight(30); lay.addWidget(self.conn_btn, 5, 0, 1, 2)

        d = self.config.get('LASER_RANGING','connection_type',fallback='serial')
        self.conn_type_combo.setCurrentText(d.title())
        self._on_conn_type_changed(d.title())
        return grp

    # ── Ranging ───────────────────────────────────────────────────────
    def _build_ranging_group(self):
        grp = QGroupBox("Ranging Controls"); lay = QGridLayout(grp)

        lay.addWidget(QLabel("Target:"), 0, 0)
        self.target_combo = QComboBox()
        self.target_combo.addItems(["First Target","Last Target","Multi-target"])
        self.target_combo.currentTextChanged.connect(self._on_target_changed)
        lay.addWidget(self.target_combo, 0, 1)

        lay.addWidget(QLabel("Freq (Hz):"), 0, 2)
        self.freq_spin = QSpinBox(); self.freq_spin.setRange(1,10); self.freq_spin.setValue(1)
        lay.addWidget(self.freq_spin, 0, 3)

        lay.addWidget(QLabel("Min gate (m):"), 1, 0)
        self.min_range_spin = QSpinBox(); self.min_range_spin.setRange(10,20000); self.min_range_spin.setValue(15)
        lay.addWidget(self.min_range_spin, 1, 1)

        lay.addWidget(QLabel("Max gate (m):"), 1, 2)
        self.max_range_spin = QSpinBox(); self.max_range_spin.setRange(10,20000); self.max_range_spin.setValue(4200)
        lay.addWidget(self.max_range_spin, 1, 3)

        self.apply_range_btn = QPushButton("Apply Gating")
        self.apply_range_btn.clicked.connect(self._apply_range_settings)
        self.apply_range_btn.setEnabled(False)
        lay.addWidget(self.apply_range_btn, 2, 2, 1, 2)

        self.single_btn = QPushButton("⚡  Single Shot")
        self.single_btn.setObjectName("btn_single")
        self.single_btn.setMinimumHeight(36); self.single_btn.setEnabled(False)
        self.single_btn.clicked.connect(self._do_single_ranging)
        lay.addWidget(self.single_btn, 3, 0, 1, 2)

        self.continuous_btn = QPushButton("▶  Start Continuous")
        self.continuous_btn.setObjectName("btn_cont")
        self.continuous_btn.setMinimumHeight(36); self.continuous_btn.setEnabled(False)
        self.continuous_btn.clicked.connect(self._toggle_continuous)
        lay.addWidget(self.continuous_btn, 3, 2, 1, 2)
        return grp

    # ── Results ───────────────────────────────────────────────────────
    def _build_results_group(self):
        grp = QGroupBox("Measurement Results"); lay = QVBoxLayout(grp)
        self.distance_label = QLabel("Distance: -- m")
        self.distance_label.setFont(QFont("Arial",20,QFont.Bold))
        self.distance_label.setAlignment(Qt.AlignCenter)
        self.distance_label.setStyleSheet(DIST_IDLE)
        lay.addWidget(self.distance_label)

        self.multi_label = QLabel()
        self.multi_label.setAlignment(Qt.AlignCenter)
        self.multi_label.setStyleSheet("background:#1a3a5a;padding:4px;border-radius:3px;color:#aaddff")
        self.multi_label.setVisible(False)
        lay.addWidget(self.multi_label)

        self.status_text = QTextEdit(); self.status_text.setMaximumHeight(120)
        self.status_text.setReadOnly(True)
        lay.addWidget(self.status_text)
        return grp

    # ── Camera ────────────────────────────────────────────────────────
    def _build_camera_group(self):
        grp = QGroupBox("IP Camera"); lay = QVBoxLayout(grp)

        ctrl = QWidget(); cl = QHBoxLayout(ctrl); cl.setContentsMargins(0,0,0,0)
        cl.addWidget(QLabel("RTSP:"))
        self.cam_url_input = QLineEdit(self.config.get('CAMERA','rtsp_url',
            fallback='rtsp://admin:12qwaszx@192.168.1.68:554/stream1'))
        self.cam_url_input.setPlaceholderText("rtsp://user:pass@ip:port/path")
        cl.addWidget(self.cam_url_input, 1)
        self.cam_btn = QPushButton("▶ Connect"); self.cam_btn.setMinimumWidth(130)
        self.cam_btn.clicked.connect(self._toggle_camera); cl.addWidget(self.cam_btn)
        lay.addWidget(ctrl)

        self.cam_label = CameraLabel()  # Используем новый специальный виджет
        self.cam_label.mouse_wheel_event.connect(self._on_camera_wheel)  # Подключаем обработчик колеса мыши
        self.cam_label.setAlignment(Qt.AlignCenter)
        self.cam_label.setStyleSheet("background:#111;color:#555;border:1px solid #333")
        self.cam_label.setMinimumSize(640, 400)
        self.cam_label.setSizePolicy(
            self.cam_label.sizePolicy().Expanding,
            self.cam_label.sizePolicy().Expanding)
        lay.addWidget(self.cam_label, 1)
        return grp

    def _on_camera_wheel(self, direction):
        """Mouse wheel zoom — uses separate ONVIF zoom controller (camera lens)."""
        if not self.zoom_controller:
            self.log_signal.emit("Zoom controller not initialized (ONVIF)")
            return
            
        try:
            # Use continuous_move with short timeout for smooth incremental zoom
            # Each wheel notch triggers a brief zoom burst, camera auto-stops after timeout
            speed = 0.5
            if direction > 0:
                self.zoom_controller.continuous_move(z=speed, timeout=0.3)
            else:
                self.zoom_controller.continuous_move(z=-speed, timeout=0.3)
        except Exception as e:
            self.log_signal.emit(f"Mouse wheel zoom failed: {e}")

    # ── Overlay controls ──────────────────────────────────────────────
    def _build_overlay_group(self):
        grp = QGroupBox("Target Overlay Controls")
        lay = QGridLayout(grp); lay.setSpacing(6)

        # --- Тип цели ---
        lay.addWidget(QLabel("Target type:"), 0, 0)
        self.ov_target_combo = QComboBox()
        self.ov_target_combo.addItems(["infantry","vehicle","aircraft","building","uav"])
        self.ov_target_combo.currentTextChanged.connect(self._ov_target_changed)
        lay.addWidget(self.ov_target_combo, 0, 1)

        # --- Прицел ---
        lay.addWidget(QLabel("Reticle:"), 0, 2)
        self.ov_reticle_combo = QComboBox()
        self.ov_reticle_combo.addItems(["crosshair","mil","bdc","circle","tactical"])
        self.ov_reticle_combo.currentTextChanged.connect(self._ov_reticle_changed)
        lay.addWidget(self.ov_reticle_combo, 0, 3)

        # --- Цвет ---
        lay.addWidget(QLabel("Color:"), 1, 0)
        self.ov_color_combo = QComboBox()
        self.ov_color_combo.addItems(list(COLORS.keys()))
        self.ov_color_combo.currentTextChanged.connect(self._ov_color_changed)
        lay.addWidget(self.ov_color_combo, 1, 1)

        # --- Масштаб ---
        lay.addWidget(QLabel("Scale:"), 1, 2)
        scale_row = QWidget(); sr = QHBoxLayout(scale_row); sr.setContentsMargins(0,0,0,0)
        self.ov_scale_slider = QSlider(Qt.Horizontal)
        self.ov_scale_slider.setRange(4, 30); self.ov_scale_slider.setValue(10)
        self.ov_scale_val = QLabel("1.0×"); self.ov_scale_val.setFixedWidth(36)
        self.ov_scale_slider.valueChanged.connect(self._ov_scale_changed)
        sr.addWidget(self.ov_scale_slider); sr.addWidget(self.ov_scale_val)
        lay.addWidget(scale_row, 1, 3)

        # --- Прозрачность ---
        lay.addWidget(QLabel("Opacity:"), 2, 0)
        op_row = QWidget(); op = QHBoxLayout(op_row); op.setContentsMargins(0,0,0,0)
        self.ov_opacity_slider = QSlider(Qt.Horizontal)
        self.ov_opacity_slider.setRange(20, 100); self.ov_opacity_slider.setValue(90)
        self.ov_opacity_val = QLabel("90%"); self.ov_opacity_val.setFixedWidth(36)
        self.ov_opacity_slider.valueChanged.connect(self._ov_opacity_changed)
        op.addWidget(self.ov_opacity_slider); op.addWidget(self.ov_opacity_val)
        lay.addWidget(op_row, 2, 1)

        # --- Яркость ---
        lay.addWidget(QLabel("Brightness:"), 2, 2)
        br_row = QWidget(); br = QHBoxLayout(br_row); br.setContentsMargins(0,0,0,0)
        self.ov_bright_slider = QSlider(Qt.Horizontal)
        self.ov_bright_slider.setRange(30, 150); self.ov_bright_slider.setValue(100)
        self.ov_bright_val = QLabel("100%"); self.ov_bright_val.setFixedWidth(40)
        self.ov_bright_slider.valueChanged.connect(self._ov_bright_changed)
        br.addWidget(self.ov_bright_slider); br.addWidget(self.ov_bright_val)
        lay.addWidget(br_row, 2, 3)

        # --- Толщина линий ---
        lay.addWidget(QLabel("Line thickness:"), 3, 0)
        lt_row = QWidget(); lt = QHBoxLayout(lt_row); lt.setContentsMargins(0,0,0,0)
        self.ov_thickness_slider = QSlider(Qt.Horizontal)
        self.ov_thickness_slider.setRange(10, 50); self.ov_thickness_slider.setValue(15)  # Умножаем на 10 для точности
        self.ov_thickness_val = QLabel("1.5×"); self.ov_thickness_val.setFixedWidth(40)
        self.ov_thickness_slider.valueChanged.connect(self._ov_thickness_changed)
        lt.addWidget(self.ov_thickness_slider); lt.addWidget(self.ov_thickness_val)
        lay.addWidget(lt_row, 3, 1, 1, 3)

        # --- Дальность (ручная, если нет дальномера) ---
        lay.addWidget(QLabel("Distance (m):"), 4, 0)
        dist_row = QWidget(); dr = QHBoxLayout(dist_row); dr.setContentsMargins(0,0,0,0)
        self.ov_dist_slider = QSlider(Qt.Horizontal)
        self.ov_dist_slider.setRange(15, 4200); self.ov_dist_slider.setValue(500)
        self.ov_dist_val = QLabel("500 m"); self.ov_dist_val.setFixedWidth(52)
        self.ov_dist_slider.valueChanged.connect(self._ov_dist_changed)
        dr.addWidget(self.ov_dist_slider); dr.addWidget(self.ov_dist_val)
        lay.addWidget(dist_row, 4, 1, 1, 3)

        # --- Чекбоксы ---
        chk_row = QWidget(); ck = QHBoxLayout(chk_row); ck.setContentsMargins(0,0,0,0)
        self.chk_target = QCheckBox("Show target silhouette"); self.chk_target.setChecked(True)
        self.chk_rings  = QCheckBox("Show range rings");        self.chk_rings.setChecked(True)
        self.chk_hud    = QCheckBox("Show HUD");                self.chk_hud.setChecked(True)
        self.chk_bar    = QCheckBox("Show distance bar");       self.chk_bar.setChecked(True)
        self.chk_gray   = QCheckBox("Grayscale mode");          self.chk_gray.setChecked(False)
        for chk, attr in [(self.chk_target,'show_target'),(self.chk_rings,'show_rings'),
                           (self.chk_hud,'show_hud'),(self.chk_bar,'show_bar'),(self.chk_gray,'gray_mode')]:
            chk.stateChanged.connect(lambda v, a=attr: setattr(self.overlay, a, bool(v)))
            ck.addWidget(chk)
        lay.addWidget(chk_row, 5, 0, 1, 4)

        return grp

    # ── Info ──────────────────────────────────────────────────────────
    def _build_info_group(self):
        grp = QGroupBox("System Information"); lay = QGridLayout(grp)
        def _btn(label, slot):
            b = QPushButton(label); b.clicked.connect(slot)
            b.setEnabled(False); b.setMinimumHeight(26); return b
        self._info_buttons = []
        btns = [
            ("Self-Check",    self._do_self_check),
            ("FPGA Version",  self._query_fpga),
            ("MCU Version",   self._query_mcu),
            ("HW Version",    self._query_hw),
            ("Serial No.",    self._query_sn),
            ("Total Pulses",  self._query_total_pulses),
            ("Session Pulses",self._query_session_pulses),
        ]
        for i, (label, slot) in enumerate(btns):
            b = _btn(label, slot); lay.addWidget(b, i//3, i%3)
            self._info_buttons.append(b)
        return grp

    # ═══════════════════════════════════════════════════════════════════
    #  Connection
    # ═══════════════════════════════════════════════════════════════════
    def _on_conn_type_changed(self, text):
        self.connection_type = text.lower()
        serial_ = (self.connection_type == "serial")
        for w, show in [('serial_port_widget',serial_),('baud_widget',serial_),('tcp_fields_widget',not serial_)]:
            if hasattr(self, w): getattr(self, w).setVisible(show)

    def _refresh_ports(self):
        import serial.tools.list_ports
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports(): self.port_combo.addItem(p.device)

    def _toggle_connection(self):
        if self.is_connected: self._disconnect()
        else: self._connect()

    def _connect(self):
        try:
            if self.connection_type == "serial":
                port = self.port_combo.currentText()
                if not port: QMessageBox.warning(self,"No Port","Select a serial port."); return
                baud = int(self.baud_combo.currentText())
                self.serial_conn = serial.Serial(port, baud, timeout=1)
                self.protocol = ProtocolHandler(self.serial_conn)
                info = f"{port} @ {baud}"
            else:
                host = self.tcp_ip_input.text().strip()
                try: tcp_port = int(self.tcp_port_input.text().strip())
                except ValueError: QMessageBox.critical(self,"Error","Invalid TCP port."); return
                self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_socket.settimeout(10)
                self.tcp_socket.connect((host, tcp_port))
                self.protocol = TcpProtocolHandler(self.tcp_socket)
                info = f"TCP {host}:{tcp_port}"
            
            self.connection_healthy = True
            self.is_connected = True
            self._update_connection_status()
            self.status_bar.showMessage(f"Connected: {info}")
            self.log_signal.emit(f"Connected: {info}")
            self._apply_target_mode()
        except Exception as e:
            QMessageBox.critical(self,"Connection Error", str(e))
            # Properly close protocol handler first, then underlying connections
            if self.protocol:
                try: self.protocol.close()
                except Exception: pass
            for x in [self.serial_conn, self.tcp_socket]:
                try:
                    if x: x.close()
                except Exception: pass
            self.serial_conn = self.tcp_socket = self.protocol = None
            self.is_connected = False
            self.connection_healthy = False

    def _disconnect(self):
        if self.is_ranging: self._stop_continuous()
        # Close protocol handler first (flushes any pending I/O)
        if self.protocol:
            try: self.protocol.close()
            except Exception: pass
        # Then close underlying connections
        for x in [self.serial_conn, self.tcp_socket]:
            try:
                if x: x.close()
            except Exception: pass
        self.serial_conn = self.tcp_socket = self.protocol = None
        self.is_connected = False
        self.connection_healthy = False
        self.continuous_measurements_active = False
        self._update_connection_status()
        self.status_bar.showMessage("Disconnected")
        self.log_signal.emit("Disconnected")

    def _update_connection_status(self):
        c = self.is_connected
        self.conn_btn.setText("Disconnect" if c else "Connect")
        self.single_btn.setEnabled(c)
        self.continuous_btn.setEnabled(c)
        self.apply_range_btn.setEnabled(c)
        for b in self._info_buttons: b.setEnabled(c)

    # ═══════════════════════════════════════════════════════════════════
    #  Ranging
    # ═══════════════════════════════════════════════════════════════════
    def _on_target_changed(self, _):
        if self.is_connected: self._apply_target_mode()

    def _apply_target_mode(self):
        if not self.protocol: return
        m = {"First Target":self.protocol.TARGET_FIRST,
             "Last Target": self.protocol.TARGET_LAST,
             "Multi-target":self.protocol.TARGET_MULTI}
        ok = self.protocol.set_target_mode(m.get(self.target_combo.currentText(), self.protocol.TARGET_FIRST))
        self.log_signal.emit(f"Target → {self.target_combo.currentText()}" + ("" if ok else " [FAILED]"))

    def _do_single_ranging(self):
        if not self.protocol or self.is_ranging: return
        r = self.protocol.single_ranging()
        if r:
            ds = f"{r['distance']:.1f}"
            self.update_signal.emit(ds, r['status_description'])
            self.log_signal.emit(f"Single: {ds} m  [{r['status_description']}]")
            if r['is_multi_target']:
                self.multi_targets[r['target_number']] = r
                self.update_multi_signal.emit(dict(self.multi_targets))
            else:
                self.update_multi_signal.emit({})
        else:
            self.log_signal.emit("Single ranging: no response")

    def _toggle_continuous(self):
        if self.is_ranging: self._stop_continuous()
        else: self._start_continuous()

    def _start_continuous(self):
        if not self.protocol: return
        ok = self.protocol.set_ranging_frequency(self.freq_spin.value())
        self.log_signal.emit(f"Freq \u2192 {self.freq_spin.value()} Hz" + ("" if ok else " [FAILED]"))
        self.is_ranging = True
        self._is_ranging_event.set()  # Signal worker thread to run
        self.continuous_measurements_active = True
        self.continuous_btn.setText("\u25a0  Stop Continuous")
        self.continuous_btn.setObjectName("btn_cont_stop")
        self.continuous_btn.setStyle(self.continuous_btn.style())
        self.single_btn.setEnabled(False)
        self.progress_bar.setVisible(True)
        self.ranging_thread = threading.Thread(target=self._continuous_worker, daemon=True)
        self.ranging_thread.start()
    
    def _stop_continuous(self):
        self.is_ranging = False
        self._is_ranging_event.clear()  # Signal worker thread to stop
        self.continuous_measurements_active = False
        if self.protocol:
            try: self.protocol.stop_ranging()
            except Exception: pass
        if self.ranging_thread and self.ranging_thread.is_alive():
            self.ranging_thread.join(timeout=3)
        self.progress_bar.setVisible(False)
        with self._multi_targets_lock:
            self.multi_targets.clear()
        self.update_multi_signal.emit({})
        QTimer.singleShot(0, self._on_continuous_stopped)
    
    def _on_continuous_stopped(self):
        self.continuous_btn.setText("\u25b6  Start Continuous")
        self.continuous_btn.setObjectName("btn_cont")
        self.continuous_btn.setStyle(self.continuous_btn.style())
        self.single_btn.setEnabled(self.is_connected)
        self.progress_bar.setVisible(False)
    
    def _continuous_worker(self):
        if not self.protocol: return
        self.protocol.start_continuous_ranging()
        consecutive_errors = 0
        max_consecutive_errors = 20
            
        while self._is_ranging_event.is_set():
            if not self.is_connected:
                self.log_signal.emit("Connection lost"); break
            try:
                raw = self.protocol.read_response(timeout=3.0)
                if raw is None:
                    consecutive_errors += 1
                    if consecutive_errors >= max_consecutive_errors: 
                        self.log_signal.emit(f"Too many consecutive timeouts ({consecutive_errors}), stopping"); break
                    continue
                parsed = self.protocol._parse_response(raw)
                if not parsed: 
                    consecutive_errors += 1
                    continue
                consecutive_errors = 0
                    
                cmd = parsed['command_code']
                if cmd == CMD_CONTINUE_RANGING and len(parsed['params']) == 4:
                    r = decode_ranging_response(parsed['params'])
                    dist_str = f"{r['distance']:.1f}"
                    if r['is_multi_target']:
                        with self._multi_targets_lock:
                            self.multi_targets[r['target_number']] = r
                            snapshot = dict(self.multi_targets)
                        self.update_multi_signal.emit(snapshot)
                    else:
                        with self._multi_targets_lock:
                            self.multi_targets.clear()
                        self.update_multi_signal.emit({})
                        self.update_signal.emit(dist_str, r['status_description'])
                    self.log_signal.emit(f"Ranging: {dist_str} m  [{r['status_description']}]")
                elif cmd == CMD_RANGING_ABNORMAL:
                    self.update_signal.emit("--", "Ranging Abnormal")
                    self.log_signal.emit("Ranging abnormal")
            except Exception as e:
                self.log_signal.emit(f"Worker error: {e}")
                consecutive_errors += 1
                if consecutive_errors >= max_consecutive_errors: 
                    self.log_signal.emit(f"Too many consecutive errors ({consecutive_errors}), stopping"); break
        # Thread-safe cleanup: signal main thread to update UI
        self.is_ranging = False
        self._is_ranging_event.clear()
        self.continuous_measurements_active = False
        QTimer.singleShot(0, self._on_continuous_stopped)

    def _apply_range_settings(self):
        if not self.protocol: return
        mn, mx = self.min_range_spin.value(), self.max_range_spin.value()
        if mn >= mx: QMessageBox.warning(self,"Invalid Range","Min must be < Max"); return
        try:
            ok1 = self.protocol.set_min_gating_distance(mn)
            ok2 = self.protocol.set_max_gating_distance(mx)
            self.log_signal.emit(f"Gating {mn}–{mx} m" + ("" if ok1 and ok2 else " [partial fail]"))
        except ValueError as e: QMessageBox.warning(self,"Range Error", str(e))

    # ═══════════════════════════════════════════════════════════════════
    #  Camera
    # ═══════════════════════════════════════════════════════════════════
    def _toggle_camera(self):
        if self.camera_thread and self.camera_thread.isRunning(): self._stop_camera()
        else: self._start_camera()

    def _start_camera(self):
        url = self.cam_url_input.text().strip()
        if not url: QMessageBox.warning(self,"Camera","Enter RTSP URL."); return
        self.cam_label.setText("Connecting…")
        self.camera_thread = CameraThread(url, self.overlay, self.config)
        self.camera_thread.frame_ready.connect(self._on_camera_frame)
        self.camera_thread.error.connect(self._on_camera_error)
        self.camera_thread.start()
        self.cam_btn.setText("■ Disconnect"); self.log_signal.emit(f"Camera: {url}")

    def _stop_camera(self):
        if self.camera_thread:
            self.camera_thread.stop()
            self.camera_thread = None
        self._last_frame_rgb = None  # Release frame buffer memory
        self.cam_label.setText("Camera not connected")
        self.cam_btn.setText("\u25b6 Connect"); self.log_signal.emit("Camera disconnected")

    def _on_camera_frame(self, frame: np.ndarray):
        # Get label dimensions for scaling
        lw = max(self.cam_label.width(),  320)
        lh = max(self.cam_label.height(), 240)
        fh, fw = frame.shape[:2]
        scale = min(lw/fw, lh/fh)
        nw, nh = int(fw*scale), int(fh*scale)
        frame_resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_NEAREST)
        rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        
        # CRITICAL: Store reference to prevent garbage collection while QImage uses it.
        # QImage(rgb.data, ...) does NOT copy the buffer; if rgb is collected,
        # the pixmap will show corrupted/garbage data.
        self._last_frame_rgb = rgb
        
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        q_img = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.cam_label.setPixmap(QPixmap.fromImage(q_img))

    def _on_camera_error(self, msg):
        self.cam_label.setText(f"Camera error:\n{msg}")
        self.cam_btn.setText("▶ Connect"); self.log_signal.emit(f"Camera error: {msg}")

    # ═══════════════════════════════════════════════════════════════════
    #  Camera & Pan-Tilt Controls (universal controller)
    # ═══════════════════════════════════════════════════════════════════
    def _camera_move_pan(self, direction):
        """Движение по оси панорамирования."""
        if self.camera_controller:
            try:
                slider_value = self.speed_slider.value()  # от 1 до 10
                base_speed = max(2.0, slider_value * 2.0)  # базовая скорость от 2 до 20 °/с (минимум 2.0)
                speed = base_speed * direction
                # Для универсального контроллера передаем нулевую скорость по наклону
                success = self.camera_controller.pan_tilt_move(speed, 0)
                if success:
                    self.log_signal.emit(f"Pan movement: {speed:+.2f}°/s")
                else:
                    self.log_signal.emit(f"Pan movement command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Pan movement failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _camera_move_tilt(self, direction):
        """Движение по оси наклона."""
        if self.camera_controller:
            try:
                slider_value = self.speed_slider.value()  # от 1 до 10
                base_speed = max(2.0, slider_value * 1.0)  # базовая скорость от 2 до 10 °/с (минимум 2.0)
                speed = base_speed * direction
                # Для универсального контроллера передаем нулевую скорость по панорамированию
                success = self.camera_controller.pan_tilt_move(0, speed)
                if success:
                    self.log_signal.emit(f"Tilt movement: {speed:+.2f}°/s")
                else:
                    self.log_signal.emit(f"Tilt movement command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Tilt movement failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _camera_move_diagonal(self, pan_direction, tilt_direction):
        """Диагональное движение."""
        if self.camera_controller:
            try:
                slider_value = self.speed_slider.value()  # от 1 до 10
                pan_base_speed = max(2.0, slider_value * 2.0)  # 2-20 °/с
                tilt_base_speed = max(2.0, slider_value * 1.0)  # 2-10 °/с
                pan_speed = pan_base_speed * pan_direction
                tilt_speed = tilt_base_speed * tilt_direction
                success = self.camera_controller.pan_tilt_move(pan_speed, tilt_speed)
                if success:
                    self.log_signal.emit(f"Diagonal movement: Pan {pan_speed:+.2f}°/s, Tilt {tilt_speed:+.2f}°/s")
                else:
                    self.log_signal.emit(f"Diagonal movement command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Diagonal movement failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _camera_stop(self):
        """Остановка движения по обеим осям."""
        if self.camera_controller:
            try:
                success = self.camera_controller.pan_tilt_stop()
                if success:
                    self.log_signal.emit("Pan-tilt stop command sent")
                else:
                    self.log_signal.emit("Pan-tilt stop command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Pan-tilt stop failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _camera_stop_pan(self):
        """Остановка движения по оси панорамирования."""
        if self.camera_controller:
            try:
                # Для универсального контроллера останавливаем обе оси, так как отдельная остановка одной оси
                # может не поддерживаться
                success = self.camera_controller.pan_tilt_stop()
                if success:
                    self.log_signal.emit("Pan stop command sent")
                else:
                    self.log_signal.emit("Pan stop command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Pan stop failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _camera_stop_tilt(self):
        """Остановка движения по оси наклона."""
        if self.camera_controller:
            try:
                # Для универсального контроллера останавливаем обе оси, так как отдельная остановка одной оси
                # может не поддерживаться
                success = self.camera_controller.pan_tilt_stop()
                if success:
                    self.log_signal.emit("Tilt stop command sent")
                else:
                    self.log_signal.emit("Tilt stop command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Tilt stop failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _camera_go_home(self):
        """Возврат в домашнюю позицию."""
        if self.camera_controller:
            try:
                # Пока просто останавливаем движение, так как возврат в домашнюю позицию
                # может отличаться для разных протоколов
                success = self.camera_controller.pan_tilt_stop()
                if success:
                    self.log_signal.emit("Go home command sent (using stop)")
                else:
                    self.log_signal.emit("Go home command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Go home failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _camera_start_self_test(self):
        """Начать самодиагностику."""
        if self.camera_controller:
            try:
                # Самодиагностика может не поддерживаться универсальным контроллером
                self.log_signal.emit("Self-test not implemented for universal controller")
            except Exception as e:
                self.log_signal.emit(f"Self-test failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    # ═══════════════════════════════════════════════════════════════════
    #  Camera Controls
    # ═══════════════════════════════════════════════════════════════════
    def _zoom_in(self):
        """Управление зумом - увеличение."""
        if self.camera_controller:
            try:
                if isinstance(self.camera_controller, OnvifCameraController):
                    speed = self.speed_slider.value() / 10.0
                    self.camera_controller.relative_move(z=speed)
                elif isinstance(self.camera_controller, PelcoDController):
                    speed = min(0x3F, max(0, self.speed_slider.value() * 5))  # Масштабируем скорость
                    self.camera_controller.zoom_in(speed)
                self.log_signal.emit("Zoom In command sent")
            except Exception as e:
                self.log_signal.emit(f"Zoom In failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _zoom_out(self):
        """Управление зумом - уменьшение."""
        if self.camera_controller:
            try:
                if isinstance(self.camera_controller, OnvifCameraController):
                    speed = self.speed_slider.value() / 10.0
                    self.camera_controller.relative_move(z=-speed)
                elif isinstance(self.camera_controller, PelcoDController):
                    speed = min(0x3F, max(0, self.speed_slider.value() * 5))  # Масштабируем скорость
                    self.camera_controller.zoom_out(speed)
                self.log_signal.emit("Zoom Out command sent")
            except Exception as e:
                self.log_signal.emit(f"Zoom Out failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _move_up(self):
        """Движение вверх."""
        if self.camera_controller:
            try:
                if isinstance(self.camera_controller, OnvifCameraController):
                    speed = self.speed_slider.value() / 10.0
                    self.camera_controller.relative_move(x=0, y=speed)
                elif isinstance(self.camera_controller, PelcoDController):
                    speed = min(0x3F, max(0, self.speed_slider.value() * 5))
                    self.camera_controller.pan_tilt_move(0, speed)
                self.log_signal.emit("Move Up command sent")
            except Exception as e:
                self.log_signal.emit(f"Move Up failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _move_down(self):
        """Движение вниз."""
        if self.camera_controller:
            try:
                if isinstance(self.camera_controller, OnvifCameraController):
                    speed = self.speed_slider.value() / 10.0
                    self.camera_controller.relative_move(x=0, y=-speed)
                elif isinstance(self.camera_controller, PelcoDController):
                    speed = min(0x3F, max(0, self.speed_slider.value() * 5))
                    self.camera_controller.pan_tilt_move(0, -speed)
                self.log_signal.emit("Move Down command sent")
            except Exception as e:
                self.log_signal.emit(f"Move Down failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _move_left(self):
        """Движение влево."""
        if self.camera_controller:
            try:
                if isinstance(self.camera_controller, OnvifCameraController):
                    speed = self.speed_slider.value() / 10.0
                    self.camera_controller.relative_move(x=-speed, y=0)
                elif isinstance(self.camera_controller, PelcoDController):
                    speed = min(0x3F, max(0, self.speed_slider.value() * 5))
                    self.camera_controller.pan_tilt_move(-speed, 0)
                self.log_signal.emit("Move Left command sent")
            except Exception as e:
                self.log_signal.emit(f"Move Left failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _move_right(self):
        """Движение вправо."""
        if self.camera_controller:
            try:
                if isinstance(self.camera_controller, OnvifCameraController):
                    speed = self.speed_slider.value() / 10.0
                    self.camera_controller.relative_move(x=speed, y=0)
                elif isinstance(self.camera_controller, PelcoDController):
                    speed = min(0x3F, max(0, self.speed_slider.value() * 5))
                    self.camera_controller.pan_tilt_move(speed, 0)
                self.log_signal.emit("Move Right command sent")
            except Exception as e:
                self.log_signal.emit(f"Move Right failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    def _stop_movement(self):
        """Остановка движения."""
        if self.camera_controller:
            try:
                if isinstance(self.camera_controller, OnvifCameraController):
                    self.camera_controller.stop_move()
                elif isinstance(self.camera_controller, PelcoDController):
                    self.camera_controller.pan_tilt_stop()
                    self.camera_controller.zoom_stop()
                self.log_signal.emit("Stop Movement command sent")
            except Exception as e:
                self.log_signal.emit(f"Stop Movement failed: {e}")
        else:
            self.log_signal.emit("Camera controller not initialized")

    # ═══════════════════════════════════════════════════════════════════
    #  Pan-Tilt Controls (new functionality)
    # ═══════════════════════════════════════════════════════════════════
    def _pan_tilt_move_pan(self, direction):
        """Движение по оси панорамирования."""
        if self.pan_tilt_controller and self.pan_tilt_controller.connected:
            try:
                # Проверяем, занята ли ось
                if self.pan_tilt_controller.is_pan_busy():
                    self.log_signal.emit("Pan axis is busy, cannot start new movement")
                    return False
                
                slider_value = self.speed_slider.value()  # от 1 до 10
                base_speed = max(2.0, slider_value * 2.0)  # базовая скорость от 2 до 20 °/с (минимум 2.0)
                speed = base_speed * direction
                success = self.pan_tilt_controller.move_pan(speed)
                if success:
                    self.log_signal.emit(f"Pan movement: {speed:+.2f}°/s")
                else:
                    self.log_signal.emit(f"Pan movement command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Pan movement failed: {e}")
        else:
            self.log_signal.emit("Pan-tilt controller not initialized or not connected")

    def _pan_tilt_move_tilt(self, direction):
        """Движение по оси наклона."""
        if self.pan_tilt_controller and self.pan_tilt_controller.connected:
            try:
                # Проверяем, занята ли ось
                if self.pan_tilt_controller.is_tilt_busy():
                    self.log_signal.emit("Tilt axis is busy, cannot start new movement")
                    return False
                
                slider_value = self.speed_slider.value()  # от 1 до 10
                base_speed = max(2.0, slider_value * 1.0)  # базовая скорость от 2 до 10 °/с (минимум 2.0)
                speed = base_speed * direction
                success = self.pan_tilt_controller.move_tilt(speed)
                if success:
                    self.log_signal.emit(f"Tilt movement: {speed:+.2f}°/s")
                else:
                    self.log_signal.emit(f"Tilt movement command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Tilt movement failed: {e}")
        else:
            self.log_signal.emit("Pan-tilt controller not initialized or not connected")

    def _pan_tilt_move_diagonal(self, pan_direction, tilt_direction):
        """Диагональное движение."""
        if self.pan_tilt_controller and self.pan_tilt_controller.connected:
            try:
                # Проверяем, заняты ли оси
                pan_busy = self.pan_tilt_controller.is_pan_busy()
                tilt_busy = self.pan_tilt_controller.is_tilt_busy()
                
                if pan_busy or tilt_busy:
                    if pan_busy and tilt_busy:
                        self.log_signal.emit("Both axes are busy, cannot start new movement")
                    elif pan_busy:
                        self.log_signal.emit("Pan axis is busy, cannot start new movement")
                    else:
                        self.log_signal.emit("Tilt axis is busy, cannot start new movement")
                    return False
                
                slider_value = self.speed_slider.value()  # от 1 до 10
                pan_base_speed = max(2.0, slider_value * 2.0)  # 2-20 °/с
                tilt_base_speed = max(2.0, slider_value * 1.0)  # 2-10 °/с
                pan_speed = pan_base_speed * pan_direction
                tilt_speed = tilt_base_speed * tilt_direction
                success = self.pan_tilt_controller.move_pan_tilt(pan_speed, tilt_speed)
                if success:
                    self.log_signal.emit(f"Diagonal movement: Pan {pan_speed:+.2f}°/s, Tilt {tilt_speed:+.2f}°/s")
                else:
                    self.log_signal.emit(f"Diagonal movement command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Diagonal movement failed: {e}")
        else:
            self.log_signal.emit("Pan-tilt controller not initialized or not connected")

    def _pan_tilt_stop(self):
        """Остановка движения по обеим осям."""
        if self.pan_tilt_controller and self.pan_tilt_controller.connected:
            try:
                success = self.pan_tilt_controller.stop_all()
                if success:
                    self.log_signal.emit("Pan-tilt stop command sent")
                else:
                    self.log_signal.emit("Pan-tilt stop command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Pan-tilt stop failed: {e}")
        else:
            self.log_signal.emit("Pan-tilt controller not initialized or not connected")

    def _pan_tilt_stop_pan(self):
        """Остановка движения по оси панорамирования."""
        if self.pan_tilt_controller and self.pan_tilt_controller.connected:
            try:
                # Просто отправляем команду остановки без проверки занятости
                success = self.pan_tilt_controller.stop_pan()
                if success:
                    self.log_signal.emit("Pan stop command sent")
                else:
                    self.log_signal.emit("Pan stop command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Pan stop failed: {e}")
        else:
            self.log_signal.emit("Pan-tilt controller not initialized or not connected")

    def _pan_tilt_stop_tilt(self):
        """Остановка движения по оси наклона."""
        if self.pan_tilt_controller and self.pan_tilt_controller.connected:
            try:
                # Просто отправляем команду остановки без проверки занятости
                success = self.pan_tilt_controller.stop_tilt()
                if success:
                    self.log_signal.emit("Tilt stop command sent")
                else:
                    self.log_signal.emit("Tilt stop command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Tilt stop failed: {e}")
        else:
            self.log_signal.emit("Pan-tilt controller not initialized or not connected")

    def _pan_tilt_go_home(self):
        """Возврат в домашнюю позицию."""
        if self.pan_tilt_controller and self.pan_tilt_controller.connected:
            try:
                success = self.pan_tilt_controller.go_to_home()
                if success:
                    self.log_signal.emit("Go home command sent")
                else:
                    self.log_signal.emit("Go home command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Go home failed: {e}")
        else:
            self.log_signal.emit("Pan-tilt controller not initialized or not connected")

    def _pan_tilt_start_self_test(self):
        """Начать самодиагностику."""
        if self.pan_tilt_controller and self.pan_tilt_controller.connected:
            try:
                success = self.pan_tilt_controller.start_self_diagnosis()
                if success:
                    self.log_signal.emit("Self-test command sent")
                else:
                    self.log_signal.emit("Self-test command failed to send")
            except Exception as e:
                self.log_signal.emit(f"Self-test failed: {e}")
        else:
            self.log_signal.emit("Pan-tilt controller not initialized or not connected")

    # ═══════════════════════════════════════════════════════════════════
    #  Overlay controls
    # ═══════════════════════════════════════════════════════════════════
    def _ov_target_changed(self, v):  self.overlay.target_type = v
    def _ov_reticle_changed(self, v): self.overlay.reticle     = v
    def _ov_color_changed(self, v):   self.overlay.color_name  = v

    def _ov_scale_changed(self, v):
        self.overlay.scale = v / 10.0
        self.ov_scale_val.setText(f"{self.overlay.scale:.1f}×")

    def _ov_opacity_changed(self, v):
        self.overlay.opacity = v / 100.0
        self.ov_opacity_val.setText(f"{v}%")

    def _ov_bright_changed(self, v):
        self.overlay.brightness = v / 100.0
        self.ov_bright_val.setText(f"{int(v)}%")

    def _ov_thickness_changed(self, v):
        self.overlay.line_thickness = v / 10.0
        self.ov_thickness_val.setText(f"{self.overlay.line_thickness:.1f}×")

    def _ov_dist_changed(self, v):
        self.overlay.distance = v
        self.ov_dist_val.setText(f"{v} m")

    # ═══════════════════════════════════════════════════════════════════
    #  Display update slots
    # ═══════════════════════════════════════════════════════════════════
    def _on_update_display(self, dist_str, status_desc):
        if dist_str != "--":
            self.distance_label.setText(f"Distance: {dist_str} m")
            self.distance_label.setStyleSheet(DIST_OK)
            # синхронизируем overlay с реальной дальностью
            try:
                self.overlay.distance = int(float(dist_str))
                self.ov_dist_slider.blockSignals(True)
                self.ov_dist_slider.setValue(self.overlay.distance)
                self.ov_dist_val.setText(f"{self.overlay.distance} m")
                self.ov_dist_slider.blockSignals(False)
            except Exception: pass
        else:
            err = "out of range" in status_desc.lower() or "abnormal" in status_desc.lower()
            self.distance_label.setText(f"Distance: --  ({status_desc})")
            self.distance_label.setStyleSheet(DIST_ERR if err else DIST_IDLE)

    def _on_log(self, msg):
        self.status_text.append(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _on_multi_targets(self, data):
        if data:
            parts = [f"#{k}: {v['distance']:.1f} m" for k,v in sorted(data.items())]
            self.multi_label.setText("Multi: " + ", ".join(parts))
            self.multi_label.setVisible(True)
        else:
            self.multi_label.setVisible(False)

    def _tick_ui(self):
        # Убираем все автоматические проверки времени последнего ответа
        # Оставляем только обновление прогресс-бара при непрерывных измерениях
        if self.is_ranging:
            self.progress_bar.setValue((self.progress_bar.value()+10)%100)
        
        # Больше не обновляем позиции поворотки, так как теперь используется универсальный контроллер
        # и позиции могут быть недоступны в некоторых режимах

    # ═══════════════════════════════════════════════════════════════════
    #  System info
    # ═══════════════════════════════════════════════════════════════════
    def _on_system_info(self, data):
        self.system_info_label.setText(data)

    # ═══════════════════════════════════════════════════════════════════
    #  Calibration
    # ═══════════════════════════════════════════════════════════════════
    def _on_calib_status(self, data):
        self.calib_status_label.setText(data)

    def _on_calib_progress(self, data):
        self.calib_progress_bar.setValue(data)

    def _on_calib_result(self, data):
        self.calib_result_label.setText(data)

    # ═══════════════════════════════════════════════════════════════════
    #  System info
    # ═══════════════════════════════════════════════════════════════════
    # ═══════════════════════════════════════════════════════════════════
    #  System info
    # ═══════════════════════════════════════════════════════════════════
    def _reset_pan_tilt_label(self):
        "Reset pan and tilt label to default state"
        self.pan_tilt_pos_label.setText("Pan: --°, Tilt: --°")

    def _do_self_check(self):
        if not self.protocol: return
        r = self.protocol.self_check()
        if r:
            self.log_signal.emit(
                f"Self-check | FPGA:{'OK' if r['fpga_ok'] else 'FAIL'} "
                f"Temp:{'OK' if r['temperature_ok'] else 'ABNORMAL'} "
                f"5V6:{'OK' if r['power_5v6_ok'] else 'FAIL'} "
                f"Echo:{r['echo_intensity']}")
        else: self.log_signal.emit("Self-check: no response")

    def _query_fpga(self):
        if not self.protocol: return
        r = self.protocol.query_fpga_version()
        if r: self.log_signal.emit(f"FPGA {r['version']} {r['year']}-{r['month']:02d}-{r['date']:02d} {r['author']}")
        else: self.log_signal.emit("FPGA: no response")

    def _query_mcu(self):
        if not self.protocol: return
        r = self.protocol.query_mcu_version()
        if r: self.log_signal.emit(f"MCU {r['version']} {r['year']}-{r['month']:02d}-{r['date']:02d} {r['author']}")
        else: self.log_signal.emit("MCU: no response")

    def _query_hw(self):
        if not self.protocol: return
        r = self.protocol.query_hardware_version()
        if r: self.log_signal.emit(f"HW MB:{r['motherboard']} CT:{r['control_board']} APD:{r['detection_board']} LD:{r['driver_board']}")
        else: self.log_signal.emit("HW: no response")

    def _query_sn(self):
        if not self.protocol: return
        r = self.protocol.query_sn_number()
        if r: self.log_signal.emit(f"SN: {r['serial_number']}  ({r['year']}-{r['month']:02d})")
        else: self.log_signal.emit("SN: no response")

    def _query_total_pulses(self):
        if not self.protocol: return
        n = self.protocol.query_total_pulses()
        self.log_signal.emit(f"Total pulses: {n}" if n else "Total pulses: no response")

    def _query_session_pulses(self):
        if not self.protocol: return
        n = self.protocol.query_session_pulses()
        self.log_signal.emit(f"Session pulses: {n}" if n else "Session pulses: no response")

    def _show_about(self):
        QMessageBox.about(self,"About",
            "<b>3km Eye-Safe Laser Rangefinder</b><br><br>"
            "1535 nm Class I · Max 4200 m · UART TTL 3.3V / TCP<br>"
            "Camera: RTSP via OpenCV · Overlay: military reticle<br><br>"
            "PyQt5 · pySerial · OpenCV")

    def closeEvent(self, e):
        # Stop ranging first (signals worker thread to exit)
        if self.is_ranging:
            self._is_ranging_event.clear()
            self._stop_continuous()
        # Disconnect from rangefinder
        if self.is_connected:
            self._disconnect()
        # Stop camera stream
        if self.camera_thread:
            self._stop_camera()
        # Disconnect camera controller (pan-tilt)
        if self.camera_controller:
            try:
                self.camera_controller.disconnect()
            except Exception:
                pass
        # Disconnect ONVIF zoom controller
        if self.zoom_controller:
            try:
                self.zoom_controller.disconnect()
            except Exception:
                pass
        # Disconnect legacy pan-tilt controller
        if hasattr(self, 'pan_tilt_controller') and self.pan_tilt_controller:
            try:
                self.pan_tilt_controller.disconnect()
            except Exception:
                pass
        e.accept()


def main():
    app = QApplication(sys.argv)
    win = LaserRangefinderApp(); win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()