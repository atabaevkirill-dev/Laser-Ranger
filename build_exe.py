"""
Скрипт для установки зависимостей и сборки исполняемого файла Laser Ranger
"""
import os
import subprocess
import sys

def install_pyinstaller():
    """Установка PyInstaller если он не установлен"""
    try:
        import PyInstaller
        print("PyInstaller уже установлен")
    except ImportError:
        print("Устанавливаю PyInstaller...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyinstaller"])
        print("PyInstaller успешно установлен")

def install_dependencies():
    """Установка всех зависимостей из requirements.txt"""
    print("Устанавливаю зависимости из requirements.txt...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])
    print("Зависимости успешно установлены")

def build_executable():
    """Сборка исполняемого файла с помощью PyInstaller"""
    print("Начинаю сборку исполняемого файла...")
    
    # Команда для PyInstaller с использованием нашего спецификационного файла
    cmd = [
        sys.executable, "-m", "PyInstaller",
        "laser_rangefinder.spec",  # Используем наш спецификационный файл
        "--onefile",  # Указываем режим сборки onefile
        "--clean",
        "--noconfirm"
    ]
    
    try:
        subprocess.check_call(cmd)
        print("Исполняемый файл успешно создан!")
        print("Файл находится в папке dist/LaserRangefinder.exe")
    except subprocess.CalledProcessError as e:
        print(f"Ошибка при сборке исполняемого файла: {e}")
        sys.exit(1)

def main():
    print("Скрипт сборки исполняемого файла Laser Ranger")
    print("="*50)
    
    # Установка зависимостей
    install_dependencies()
    
    # Установка PyInstaller
    install_pyinstaller()
    
    # Сборка исполняемого файла
    build_executable()
    
    print("="*50)
    print("Процесс сборки завершен!")

if __name__ == "__main__":
    main()