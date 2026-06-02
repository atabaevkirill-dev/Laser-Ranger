# Архитектурное описание системы лазерного дальномера

## Общая архитектура

Система Laser Ranger построена по модульной архитектуре с четким разделением ответственности между компонентами. Архитектура следует принципам MVC (Model-View-Controller) с дополнительными слоями для протоколирования и управления устройствами.

```
┌─────────────────────────────────────────────────────────────────┐
│                        Presentation Layer                       │
│  ┌─────────────────┐  ┌──────────────────────────────────────┐ │
│  │   GUI (PyQt5)   │  │     Overlay Renderer                 │ │
│  │                 │  │  ┌─────────────────────────────────┐ │ │
│  │  Main Window    │  │  │   Target Silhouettes          │ │ │
│  │                 │  │  │  • Infantry, Vehicle, Aircraft │ │ │
│  │  Controls       │  │  │  • Building, UAV              │ │ │
│  │                 │  │  └─────────────────────────────────┘ │ │
│  │  Status Bar     │  │  ┌─────────────────────────────────┐ │ │
│  │                 │  │  │   Reticle Styles              │ │ │
│  │  Camera View    │  │  │  • Crosshair, MIL-DOT, BDC    │ │ │
│  │                 │  │  │  • Circle, Tactical           │ │ │
│  │  etc.           │  │  └─────────────────────────────────┘ │ │
│  └─────────────────┘  └──────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Core Logic Layer                           │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │  Main Controller│  │   Data Fusion   │  │   Protocol      │ │
│  │     (main.py)   │  │     Engine      │  │   Handler       │ │
│  │                 │  │  ┌─────────────┐│  │  ┌───────────┐  │ │
│  │  • Camera       │  │  │ Laser Data ││  │  │ Serial    │  │ │
│  │    Control      │  │  │ Sync       ││  │  │ TCP/IP    │  │ │
│  │  • UI Events    │  │  │ Distance   ││  │  │ ONVIF     │  │ │
│  │  • Settings     │  │  │ Overlay    ││  │  └───────────┘  │ │
│  │    Management   │  │  └─────────────┘│  │                 │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Hardware Interface Layer                     │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │  Camera         │  │  Laser Rangefinder│ │  Network      │ │
│  │  Control        │  │  Interface      │  │  Services     │ │
│  │                 │  │                 │  │                 │ │
│  │  • ONVIF        │  │  • Serial       │  │  • TCP Server │ │
│  │  • RTSP Stream  │  │  • Data Parsing │  │  • Remote API │ │
│  │  • PTZ Control  │  │  • Calibration  │  │  • Status     │ │
│  │                 │  │                 │  │    Reporting   │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Компонентное описание

### [main.py](file:///C:/Users/White_/Desktop/Laser%20Ranger/main.py) - Контроллер приложения
- Главный класс [LaserRangefinderApp](file:///C:/Users/White_/Desktop/Laser%20Ranger/main.py#L108-L868) управляет жизненным циклом приложения
- Создает и управляет интерфейсом пользователя
- Координирует взаимодействие между различными компонентами
- Обрабатывает события пользователя и передает их соответствующим подсистемам

### [overlay_renderer.py](file:///C:/Users/White_/Desktop/Laser%20Ranger/overlay_renderer.py) - Компонент визуализации
- Класс [OverlayState](file:///C:/Users/White_/Desktop/Laser%20Ranger/overlay_renderer.py#L275-L291) хранит состояние всех параметров оверлея
- Функция [render](file:///C:/Users/White_/Desktop/Laser%20Ranger/overlay_renderer.py#L293-L348) выполняет наложение графических элементов на кадр видео
- Поддерживает различные стили прицелов и силуэты целей
- Включает в себя функционал градаций серого и регулировки толщины линий

### [onvif_camera_control.py](file:///C:/Users/White_/Desktop/Laser%20Ranger/onvif_camera_control.py) - Контроллер камеры
- Класс [OnvifCameraController](file:///C:/Users/White_/Desktop/Laser%20Ranger/onvif_camera_control.py#L16-L201) обеспечивает управление ONVIF-совместимыми камерами
- Поддерживает PTZ (панорамирование, наклон, масштабирование) операции
- Класс [PelcoDController](file:///C:/Users/White_/Desktop/Laser%20Ranger/onvif_camera_control.py#L204-L257) для управления камерами через протокол Pelco-D

### [protocol_handler.py](file:///C:/Users/White_/Desktop/Laser%20Ranger/protocol_handler.py) и [tcp_protocol_handler.py](file:///C:/Users/White_/Desktop/Laser%20Ranger/tcp_protocol_handler.py) - Протокольные обработчики
- Обеспечивают связь с лазерным дальномером
- Парсят данные из дальномера и передают их в основное приложение
- Обрабатывают команды и запросы к дальномеру

## Расширения функционала

### Режим градаций серого
- Реализован в классе [OverlayState](file:///C:/Users/White_/Desktop/Laser%20Ranger/overlay_renderer.py#L275-L291) через параметр [gray_mode](file:///C:/Users/White_/Desktop/Laser%20Ranger/main.py#L408-L408)
- Использует OpenCV функцию `cv2.cvtColor` для преобразования изображения
- Интегрирован в интерфейс как чекбокс "Grayscale mode"

### Регулировка толщины линий
- Реализована через параметр [line_thickness](file:///C:/Users/White_/Desktop/Laser%20Ranger/overlay_renderer.py#L293-L348) в классе [OverlayState](file:///C:/Users/White_/Desktop/Laser%20Ranger/overlay_renderer.py#L275-L291)
- Новый расчет толщины линий: `lw = max(1, int(st.line_thickness * st.scale))`
- Добавлен слайдер в интерфейс с диапазоном от 1.0× до 5.0×

## Потоки выполнения

### Поток камеры
1. [CameraThread](file:///C:/Users/White_/Desktop/Laser%20Ranger/main.py#L85-L108) захватывает кадры из RTSP-потока
2. Каждый кадр передается в функцию [render](file:///C:/Users/White_/Desktop/Laser%20Ranger/overlay_renderer.py#L293-L348) для наложения оверлея
3. Результат отправляется сигналом [frame_ready](file:///C:/Users/White_/Desktop/Laser%20Ranger/main.py#L86-L86) в главный поток GUI
4. Изображение отображается в [cam_label](file:///C:/Users/White_/Desktop/Laser%20Ranger/main.py#L186-L186)

### Поток данных дальномера
1. [ProtocolHandler](file:///C:/Users/White_/Desktop/Laser%20Ranger/protocol_handler.py#L1-L11) получает данные из последовательного порта или TCP-соединения
2. Данные парсятся и извлекается информация о расстоянии
3. Результат передается в главное окно через сигналы
4. Данные отображаются и могут использоваться для обновления оверлея

## Системные требования

- Python 3.8+
- PyQt5 >= 5.15
- opencv-python-headless >= 4.5
- numpy >= 1.21.0
- pyserial >= 3.5
- onvif_zeep >= 0.2.12

## Конфигурация

Система использует файл [config.ini](file:///C:/Users/White_/Desktop/Laser%20Ranger/config.ini) для хранения настроек:
- Параметры подключения к дальномеру
- Настройки камеры (IP, учетные данные)
- Параметры ONVIF
- Настройки пользовательского интерфейса