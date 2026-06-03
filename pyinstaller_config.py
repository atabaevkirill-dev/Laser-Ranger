"""
Дополнительные настройки и хуки для PyInstaller
"""

# Хуки для корректной сборки зависимостей
from PyInstaller.utils.hooks import collect_data_files, collect_submodules

# Сборка подмодулей для onvif_zeep
hiddenimports = [
    'onvif.soap',
    'onvif.wdsl',
    'onvif.services',
    'onvif.util',
    'zeep',
    'zeep.transports',
    'zeep.wsdl',
    'zeep.wsdl.suds',
    'zeep.xsd',
    'zeep.xsd.builtins',
    'zeep.wsse',
    'zeep.plugins',
    'zeep.cache',
    'urllib3',
    'urllib3.contrib.pyopenssl',
    'idna',
    'idna.idnadata',
    'serial.tools.list_ports',
    'serial.tools.list_ports_common',
    'serial.tools.list_ports_linux',
    'serial.tools.list_ports_osx',
    'serial.tools.list_ports_windows',
    'numpy.core._methods',
    'numpy.lib.format',
    'cv2',
    'PyQt5.sip',
    'PyQt5.QtCore',
    'PyQt5.QtGui',
    'PyQt5.QtWidgets',
    'configparser',
    'math',
    'threading',
    'socket',
    'time',
]

# Данные для включения
datas = [
    ('config.ini', '.'),
]

# Бинарные файлы для включения (если необходимо)
binaries = []

# Исключения (если необходимо исключить какие-то модули)
excludes = [
    'tkinter',
    'matplotlib',
    'scipy',  # Если не используется
]

def get_hook_dirs():
    """Возвращает список директорий с хуками"""
    import os
    return [os.path.dirname(__file__)]