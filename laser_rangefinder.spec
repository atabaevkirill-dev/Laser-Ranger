# -*- mode: python ; coding: utf-8 -*-
"""
Файл спецификации для PyInstaller для приложения Laser Ranger
"""

import os
from pyinstaller_config import hiddenimports, datas, binaries, excludes

block_cipher = None

a = Analysis(
    ['main.py'],
    pathex=['.'],  # Используем текущую директорию
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=excludes,
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

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
    console=False,  # Устанавливаем в False для GUI приложения
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=None,  # Можно добавить путь к иконке, если она есть
    onefile=True,  # Устанавливаем в True для создания одного файла
)