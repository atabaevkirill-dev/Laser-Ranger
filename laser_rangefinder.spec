# -*- mode: python ; coding: utf-8 -*-

"""
Упрощенный файл спецификации PyInstaller для приложения Laser Rangefinder
"""

# Используем простую команду PyInstaller без сложных структур
a = Analysis(
    ['main.py'],
    pathex=[],
    binaries=[],
    datas=[('config.ini', '.')],  # Только необходимые данные
    hiddenimports=[
        'serial.tools.list_ports',
        'onvif_zeep',
        'onvif_camera_control',
        'pan_tilt_controller',
        'protocol_handler',
        'tcp_protocol_handler',
        'protocol_base',
        'overlay_renderer',
        'PyQt5.sip',
        'numpy.core._methods',
        'numpy.lib.format',
        'cv2',
        'PyQt5.QtCore',
        'PyQt5.QtGui',
        'PyQt5.QtWidgets',
        'configparser',
        'math',
        'threading',
        'socket',
        'time',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=None,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=None)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='LaserRangefinder',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='LaserRangefinder',
)