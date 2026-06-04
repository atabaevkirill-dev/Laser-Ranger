"""
Pan-tilt controller class using a custom TCP protocol.
"""
import time
import socket
import re
from typing import Tuple, Optional


class PanTiltController:
    def __init__(self, host: str = "192.168.1.115", port: int = 9760):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        # Добавляем настройки инверсии осей
        self.invert_pan = False
        self.invert_tilt = False  # По умолчанию tilt не инвертирован

    def connect(self) -> bool:
        """Подключиться к устройству поворотки"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((self.host, self.port))
            self.connected = True
            return True
        except Exception as e:
            print(f"Ошибка подключения: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Отключиться от устройства поворотки"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.connected = False

    def send_command(self, command: str) -> Optional[str]:
        """Отправить команду и получить ответ"""
        if not self.connected or not self.socket:
            return None

        try:
            self.socket.sendall(command.encode('utf-8'))
            response = self.socket.recv(1024).decode('utf-8').strip()
            return response
        except Exception as e:
            print(f"Ошибка отправки команды {command}: {e}")
            self.connected = False
            return None

    def get_firmware_type(self) -> Optional[str]:
        """Получить тип прошивки"""
        response = self.send_command("$Io#")
        return response if response else None

    def get_firmware_version(self) -> Optional[float]:
        """Получить версию прошивки"""
        response = self.send_command("$V#")
        if response and response.startswith("$V") and response.endswith("#"):
            try:
                hex_part = response[2:-1]  # Извлекаем шестнадцатеричную часть
                version_int = int(hex_part, 16)
                return version_int / 100.0
            except:
                return None
        return None

    def get_pan_speeds(self) -> Optional[Tuple[float, float, float]]:
        """Получить скорости для оси поворота (min, accDec, max)"""
        response = self.send_command("$3#")
        if response and response.startswith("$3,") and response.endswith("#"):
            try:
                parts = response[3:-1].split(",")
                if len(parts) == 3:
                    return tuple(float(x) for x in parts)
            except:
                pass
        return None

    def set_pan_speeds(self, min_speed: float, acc_dec: float, max_speed: float) -> bool:
        """Задать скорости для оси поворота"""
        command = f"$3,{min_speed:.2f},{acc_dec:.2f},{max_speed:.2f}#"
        response = self.send_command(command)
        return response is not None

    def get_tilt_speeds(self) -> Optional[Tuple[float, float, float]]:
        """Получить скорости для оси наклона (min, accDec, max)"""
        response = self.send_command("$4#")
        if response and response.startswith("$4,") and response.endswith("#"):
            try:
                parts = response[3:-1].split(",")
                if len(parts) == 3:
                    return tuple(float(x) for x in parts)
            except:
                pass
        return None

    def set_tilt_speeds(self, min_speed: float, acc_dec: float, max_speed: float) -> bool:
        """Задать скорости для оси наклона"""
        command = f"$4,{min_speed:.2f},{acc_dec:.2f},{max_speed:.2f}#"
        response = self.send_command(command)
        return response is not None

    def get_pan_limits(self) -> Optional[Tuple[int, float, float]]:
        """Получить ограничения для оси поворота (enable, left, right)"""
        response = self.send_command("$7#")
        if response and response.startswith("$7,") and response.endswith("#"):
            try:
                parts = response[3:-1].split(",")
                if len(parts) == 3:
                    return int(parts[0]), float(parts[1]), float(parts[2])
            except:
                pass
        return None

    def set_pan_limits(self, enable: int, left: float, right: float) -> bool:
        """Задать ограничения для оси поворота"""
        command = f"$7,{enable},{left:.2f},{right:.2f}#"
        response = self.send_command(command)
        return response is not None

    def get_tilt_limits(self) -> Optional[Tuple[float, float]]:
        """Получить ограничения для оси наклона (left, right)"""
        response = self.send_command("$8#")
        if response and response.startswith("$8,") and response.endswith("#"):
            try:
                parts = response[3:-1].split(",")
                if len(parts) == 2:
                    return tuple(float(x) for x in parts)
            except:
                pass
        return None

    def set_tilt_limits(self, left: float, right: float) -> bool:
        """Задать ограничения для оси наклона"""
        command = f"$8,{left:.2f},{right:.2f}#"
        response = self.send_command(command)
        return response is not None

    def get_current_pan_position(self) -> Optional[float]:
        """Получить текущую позицию оси поворота"""
        response = self.send_command("$o#")
        if response and response.startswith("$o,") and response.endswith("#"):
            try:
                # Извлекаем только содержимое между $o, и #
                pos_str = response.split("$o,")[1].split("#")[0]
                pos = float(pos_str)
                return pos
            except (ValueError, IndexError) as e:
                print(f"Ошибка при разборе ответа для позиции pan: {e}")
                return None
        return None

    def get_current_tilt_position(self) -> Optional[float]:
        """Получить текущую позицию оси наклона"""
        response = self.send_command("$O#")
        if response and response.startswith("$O,") and response.endswith("#"):
            try:
                # Извлекаем только содержимое между $O, и #
                pos_str = response.split("$O,")[1].split("#")[0]
                pos = float(pos_str)
                return pos
            except (ValueError, IndexError) as e:
                print(f"Ошибка при разборе ответа для позиции tilt: {e}")
                return None
        return None

    def move_pan(self, speed: float) -> bool:
        """Задать скорость поворота (движение по оси X), ограничивая в пределах допустимого диапазона"""
        # Проверяем, занята ли ось поворота
        if self.is_pan_busy():
            print("Ось поворота занята выполнением предыдущей команды")
            return False
            
        # Применяем инверсию, если нужно
        if self.invert_pan:
            speed = -speed
        
        # Ограничиваем скорость в пределах от -50.0 до 50.0 °/с согласно спецификации
        # Убедимся, что модуль скорости не слишком маленький, иначе будет остановка
        abs_speed = abs(speed)
        if abs_speed < 0.5:  # Минимальная скорость для запуска движения
            abs_speed = 0.5
        
        # Сохраняем знак исходной скорости
        sign = 1 if speed >= 0 else -1
        signed_speed = sign * abs_speed
        limited_speed = max(-50.0, min(50.0, signed_speed))
        
        # На основе наблюдений: устройство инвертирует все команды
        # Для компенсации всегда инвертируем команду
        corrected_speed = -limited_speed
        
        command = f"$w,{corrected_speed:+.2f}#"  # Правильный формат: $w, targetSpeed#
        print(f"Отправка команды pan: {command}")  # Отладочное сообщение
        response = self.send_command(command)
        print(f"Ответ на команду pan: {response}")  # Отладочное сообщение
        return response is not None

    def move_tilt(self, speed: float) -> bool:
        """Задать скорость наклона (движение по оси Y), ограничивая в пределах допустимого диапазона"""
        # Проверяем, занята ли ось наклона
        if self.is_tilt_busy():
            print("Ось наклона занята выполнением предыдущей команды")
            return False
            
        # Применяем инверсию, если нужно
        if self.invert_tilt:
            speed = -speed
        
        # Ограничиваем скорость в пределах от -17.0 до 17.0 °/с согласно спецификации
        # Убедимся, что модуль скорости не слишком маленький, иначе будет остановка
        abs_speed = abs(speed)
        if abs_speed < 0.5:  # Минимальная скорость для запуска движения
            abs_speed = 0.5
        
        # Сохраняем знак исходной скорости
        sign = 1 if speed >= 0 else -1
        signed_speed = sign * abs_speed
        limited_speed = max(-17.0, min(17.0, signed_speed))
        
        # На основе наблюдений: устройство инвертирует все команды
        # Для компенсации всегда инвертируем команду
        corrected_speed = -limited_speed
        
        command = f"$W,{corrected_speed:+.2f}#"  # Правильный формат: $W, targetSpeed#
        print(f"Отправка команды tilt: {command}")  # Отладочное сообщение
        response = self.send_command(command)
        print(f"Ответ на команду tilt: {response}")  # Отладочное сообщение
        return response is not None

    def move_pan_tilt(self, pan_speed: float, tilt_speed: float) -> bool:
        """Simultaneous movement on both axes."""
        if self.is_pan_busy():
            print("Pan axis is busy")
            return False
            
        if self.is_tilt_busy():
            print("Tilt axis is busy")
            return False
            
        success_pan = self.move_pan(pan_speed)
        time.sleep(0.05)  # 50ms delay for command processing
        success_tilt = self.move_tilt(tilt_speed)
        return success_pan and success_tilt

    def stop_pan(self) -> bool:
        """Остановить движение по оси поворота"""
        response = self.send_command("$u#")
        return response is not None

    def stop_tilt(self) -> bool:
        """Остановить движение по оси наклона"""
        response = self.send_command("$U#")
        return response is not None

    def stop_all(self) -> bool:
        """Остановить движение по обеим осям"""
        success_pan = self.stop_pan()
        success_tilt = self.stop_tilt()
        return success_pan and success_tilt

    def go_to_pan_position(self, position: float, max_speed: Optional[float] = None) -> bool:
        """Перейти в заданную позицию по оси поворота"""
        if max_speed is not None:
            command = f"$x,{position:.2f},{max_speed:.2f}#"  # Правильный формат: $x, targetPos, maxSpeed#
        else:
            command = f"$x,{position:.2f}#"  # Правильный формат: $x, targetPos#
        response = self.send_command(command)
        return response is not None

    def go_to_tilt_position(self, position: float, max_speed: Optional[float] = None) -> bool:
        """Перейти в заданную позицию по оси наклона"""
        if max_speed is not None:
            command = f"$X,{position:.2f},{max_speed:.2f}#"  # Правильный формат: $X, targetPos, maxSpeed#
        else:
            command = f"$X,{position:.2f}#"  # Правильный формат: $X, targetPos#
        response = self.send_command(command)
        return response is not None

    def go_to_home(self) -> bool:
        """Вернуться в домашнюю позицию (0,0)"""
        success_pan = self.go_to_pan_position(0.0)
        success_tilt = self.go_to_tilt_position(0.0)
        return success_pan and success_tilt

    def start_self_diagnosis(self) -> bool:
        """Начать самодиагностику для обеих осей"""
        success_pan = self.send_command("$m,1#") is not None
        success_tilt = self.send_command("$M,1#") is not None
        return success_pan and success_tilt

    def get_pan_state(self) -> Optional[int]:
        """Получить состояние оси поворота"""
        response = self.send_command("$m#")
        if response and response.startswith("$m,") and response.endswith("#"):
            try:
                state = int(response[3:-1])
                return state
            except:
                pass
        return None

    def get_tilt_state(self) -> Optional[int]:
        """Получить состояние оси наклона"""
        response = self.send_command("$M#")
        if response and response.startswith("$M,") and response.endswith("#"):
            try:
                state = int(response[3:-1])
                return state
            except:
                pass
        return None

    def is_pan_busy(self) -> bool:
        """Проверить, занята ли ось поворота выполнением команды"""
        response = self.send_command("$q#")
        if response and response.startswith("$q,") and response.endswith("#"):
            try:
                busy_status = int(response[3:-1])  # 0-удержание, 1-разгон, 2-торможение, 3-равномерное движение
                return busy_status != 0  # Занято, если не в состоянии удержания позиции
            except:
                return False  # В случае ошибки считаем, что не занято
        return False

    def is_tilt_busy(self) -> bool:
        """Проверить, занята ли ось наклона выполнением команды"""
        response = self.send_command("$Q#")
        if response and response.startswith("$Q,") and response.endswith("#"):
            try:
                busy_status = int(response[3:-1])  # 0-удержание, 1-разгон, 2-торможение, 3-равномерное движение
                return busy_status != 0  # Занято, если не в состоянии удержания позиции
            except:
                return False  # В случае ошибки считаем, что не занято
        return False