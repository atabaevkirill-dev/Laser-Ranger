"""
overlay_renderer.py
Военный прицел поверх кадра камеры — чистый OpenCV, без Qt.
Вызывается из CameraThread перед отправкой frame_ready.
"""

import cv2
import numpy as np
import math
import time

# ─── Цветовые пресеты (BGR) ───────────────────────────────────────────────
COLORS = {
    'green':  (0,   255,  68),
    'red':    (0,    51, 255),
    'yellow': (0,   204, 255),
    'cyan':   (255, 204,   0),
    'white':  (255, 255, 255),
    'orange': (0,   136, 255),
}

# ─── Силуэты целей ────────────────────────────────────────────────────────

def _draw_infantry(img, cx, cy, scale):
    s = scale
    # корпус
    pts_body = np.array([
        [cx-int(18*s), cy+int(60*s)],
        [cx-int(22*s), cy-int(10*s)],
        [cx+int(22*s), cy-int(10*s)],
        [cx+int(18*s), cy+int(60*s)],
    ], np.int32)
    cv2.fillPoly(img, [pts_body], (45, 65, 40))
    cv2.polylines(img, [pts_body], True, (60, 90, 55), 1)
    # голова + шлем
    cv2.ellipse(img, (cx, cy-int(28*s)), (int(14*s), int(16*s)), 0, 0, 360, (55, 80, 50), -1)
    cv2.ellipse(img, (cx, cy-int(32*s)), (int(17*s), int(12*s)), 0, 180, 360, (35, 55, 30), -1)
    # ноги
    cv2.rectangle(img, (cx-int(20*s), cy+int(60*s)), (cx-int(5*s),  cy+int(110*s)), (38, 55, 33), -1)
    cv2.rectangle(img, (cx+int(5*s),  cy+int(60*s)), (cx+int(20*s), cy+int(110*s)), (38, 55, 33), -1)
    # оружие
    cv2.rectangle(img, (cx+int(22*s), cy-int(5*s)), (cx+int(26*s), cy+int(50*s)), (55, 55, 55), -1)


def _draw_vehicle(img, cx, cy, scale):
    s = scale
    # корпус
    cv2.rectangle(img, (cx-int(90*s), cy-int(28*s)), (cx+int(90*s), cy+int(35*s)), (42, 65, 30), -1)
    cv2.rectangle(img, (cx-int(90*s), cy-int(28*s)), (cx+int(90*s), cy+int(35*s)), (60, 90, 45), 1)
    # башня
    cv2.ellipse(img, (cx-int(10*s), cy-int(28*s)), (int(45*s), int(28*s)), 0, 0, 360, (30, 50, 20), -1)
    # пушка
    angle = -0.2
    gx = int(cx - 10*s + math.cos(angle)*110*s)
    gy = int(cy - 28*s + math.sin(angle)*110*s)
    cv2.line(img, (cx-int(10*s), cy-int(28*s)), (gx, gy), (50, 50, 50), int(5*s))
    # катки
    for ox in range(-80, 90, 40):
        cv2.circle(img, (cx+int(ox*s), cy+int(42*s)), int(18*s), (40, 40, 40), -1)
        cv2.circle(img, (cx+int(ox*s), cy+int(42*s)), int(10*s), (60, 60, 60), -1)
    # гусеница
    cv2.rectangle(img, (cx-int(90*s), cy+int(30*s)), (cx+int(90*s), cy+int(52*s)), (30, 30, 30), -1)


def _draw_aircraft(img, cx, cy, scale):
    s = scale
    # фюзеляж
    cv2.ellipse(img, (cx, cy), (int(110*s), int(18*s)), 0, 0, 360, (70, 70, 100), -1)
    # крылья L
    wing_l = np.array([
        [cx-int(20*s), cy],
        [cx-int(130*s), cy+int(65*s)],
        [cx-int(85*s),  cy+int(65*s)],
        [cx+int(25*s),  cy-int(12*s)],
    ], np.int32)
    cv2.fillPoly(img, [wing_l], (60, 60, 90))
    # крылья R
    wing_r = np.array([
        [cx-int(20*s), cy],
        [cx+int(100*s), cy+int(50*s)],
        [cx+int(70*s),  cy+int(50*s)],
        [cx+int(25*s),  cy-int(12*s)],
    ], np.int32)
    cv2.fillPoly(img, [wing_r], (60, 60, 90))
    # хвост
    tail = np.array([
        [cx-int(95*s), cy],
        [cx-int(110*s), cy-int(50*s)],
        [cx-int(75*s),  cy-int(18*s)],
    ], np.int32)
    cv2.fillPoly(img, [tail], (55, 55, 85))
    # кабина
    cv2.ellipse(img, (cx+int(70*s), cy-int(7*s)), (int(18*s), int(12*s)), 0, 0, 360, (30, 60, 90), -1)


def _draw_building(img, cx, cy, scale):
    s = scale
    bw, bh = int(120*s), int(160*s)
    cv2.rectangle(img, (cx-bw//2, cy-bh), (cx+bw//2, cy+int(5*s)), (85, 80, 70), -1)
    cv2.rectangle(img, (cx-bw//2, cy-bh), (cx+bw//2, cy-bh+int(12*s)), (65, 60, 50), -1)
    # окна
    ww, wh = int(18*s), int(20*s)
    for row in range(4):
        for col in range(3):
            wx = cx - bw//2 + int(20*s) + col*int(38*s)
            wy = cy - bh + int(22*s) + row*int(38*s)
            bright = np.random.choice([True, False], p=[0.7, 0.3])
            wc = (40, 190, 230) if bright else (20, 60, 100)
            cv2.rectangle(img, (wx, wy), (wx+ww, wy+wh), wc, -1)
    # дверь
    cv2.rectangle(img, (cx-int(15*s), cy-int(30*s)), (cx+int(15*s), cy+int(5*s)), (40, 30, 20), -1)


def _draw_uav(img, cx, cy, scale):
    s = scale
    arm_len = int(75*s)
    for ax, ay in [(-1,-1),(-1,1),(1,-1),(1,1)]:
        ex = cx + int(ax * arm_len * 0.7)
        ey = cy + int(ay * arm_len * 0.7)
        cv2.line(img, (cx, cy), (ex, ey), (60, 60, 70), int(3*s))
        # диск ротора
        cv2.ellipse(img, (ex, ey), (int(22*s), int(8*s)), 45*((ax+ay+2)//2), 0, 360,
                    (80, 80, 160), 1)
    # корпус
    cv2.ellipse(img, (cx, cy), (int(22*s), int(15*s)), 0, 0, 360, (30, 30, 50), -1)
    cv2.ellipse(img, (cx, cy), (int(22*s), int(15*s)), 0, 0, 360, (70, 70, 110), 1)
    # камера
    cv2.circle(img, (cx, cy), int(8*s), (20, 20, 30), -1)
    cv2.circle(img, (cx, cy), int(4*s), (40, 80, 120), -1)


TARGET_DRAW = {
    'infantry': _draw_infantry,
    'vehicle':  _draw_vehicle,
    'aircraft': _draw_aircraft,
    'building': _draw_building,
    'uav':      _draw_uav,
}

TARGET_LABELS = {
    'infantry': 'Infantry',
    'vehicle':  'Armored vehicle',
    'aircraft': 'Fixed-wing aircraft',
    'building': 'Building / Structure',
    'uav':      'UAV / Drone',
}

# ─── Прицелы ──────────────────────────────────────────────────────────────

def _reticle_crosshair(img, cx, cy, R, col, lw):
    gap = int(R * 0.12)
    arm = int(R * 0.55)
    for dx, dy in [(0,-1),(0,1),(-1,0),(1,0)]:
        cv2.line(img, (cx+dx*gap, cy+dy*gap), (cx+dx*arm, cy+dy*arm), col, lw)
    # стадия
    for off in [0.2, 0.35, 0.52]:
        ow = int(R * 0.55)
        oy = int(R * off)
        cv2.line(img, (cx-ow, cy+oy), (cx+ow, cy+oy), col, max(1, lw-1))
        cv2.line(img, (cx-ow, cy-oy), (cx+ow, cy-oy), col, max(1, lw-1))
    cv2.circle(img, (cx, cy), int(R*0.05), col, lw)


def _reticle_mil(img, cx, cy, R, col, lw):
    gap = int(R*0.1)
    arm = int(R*0.48)
    for dx, dy in [(0,-1),(0,1),(-1,0),(1,0)]:
        cv2.line(img, (cx+dx*gap, cy+dy*gap), (cx+dx*arm, cy+dy*arm), col, lw)
    step = int(R * 0.18)
    for row in range(-3, 4):
        for col_ in range(-3, 4):
            if abs(row) < 1 and abs(col_) < 1:
                continue
            if row == 0 or col_ == 0:
                cv2.circle(img, (cx+col_*step, cy+row*step), max(2, int(R*0.025)), col, -1)
    cv2.circle(img, (cx, cy), int(R*0.04), col, lw)


def _reticle_bdc(img, cx, cy, R, col, lw):
    arm = int(R*0.5)
    cv2.line(img, (cx-arm, cy), (cx+arm, cy), col, lw)
    cv2.line(img, (cx, cy-arm), (cx, cy+int(R*0.25)), col, lw)
    # дроп-метки
    for i, (off, bw_) in enumerate([(0.18,0.38),(0.30,0.30),(0.42,0.22),(0.55,0.14)]):
        oy = int(R*off)
        bw = int(R*bw_)
        cv2.line(img, (cx-bw, cy+oy), (cx+bw, cy+oy), col, max(1, lw-1))
        cv2.putText(img, str(i+3), (cx+bw+4, cy+oy+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)
    # треугольник вверх
    pts = np.array([[cx,cy-arm//2],[cx-int(R*0.07),cy-int(R*0.18)],[cx+int(R*0.07),cy-int(R*0.18)]], np.int32)
    cv2.fillPoly(img, [pts], col)


def _reticle_circle(img, cx, cy, R, col, lw):
    cv2.circle(img, (cx, cy), int(R*0.48), col, lw)
    cv2.circle(img, (cx, cy), int(R*0.22), col, max(1, lw-1))
    for a in range(0, 360, 30):
        rad = math.radians(a)
        thick = lw if a % 90 == 0 else max(1, lw-1)
        r1, r2 = int(R*0.4), int(R*(0.32 if a%90==0 else 0.36))
        x1 = cx + int(math.cos(rad)*r1); y1 = cy + int(math.sin(rad)*r1)
        x2 = cx + int(math.cos(rad)*r2); y2 = cy + int(math.sin(rad)*r2)
        cv2.line(img, (x1,y1), (x2,y2), col, thick)
    arm = int(R*0.7)
    for dx, dy in [(0,-1),(0,1),(-1,0),(1,0)]:
        cv2.line(img, (cx+dx*int(R*0.52), cy+dy*int(R*0.52)), (cx+dx*arm, cy+dy*arm), col, max(1,lw-1))
    cv2.circle(img, (cx, cy), int(R*0.03), col, -1)


def _reticle_tactical(img, cx, cy, R, col, lw):
    bs = int(R*0.58)
    bl = int(R*0.17)
    for sx, sy in [(-1,-1),(-1,1),(1,-1),(1,1)]:
        p = (cx+sx*bs, cy+sy*bs)
        cv2.line(img, p, (cx+sx*bs, cy+sy*(bs-bl)), col, lw)
        cv2.line(img, p, (cx+sx*(bs-bl), cy+sy*bs), col, lw)
    mi = int(R*0.06)
    cv2.line(img, (cx-mi, cy), (cx+mi, cy), col, max(1,lw-1))
    cv2.line(img, (cx, cy-mi), (cx, cy+mi), col, max(1,lw-1))
    cv2.circle(img, (cx, cy), int(R*0.03), col, -1)
    for a in [45, 135, 225, 315]:
        rad = math.radians(a)
        r1, r2 = int(R*0.18), int(R*0.36)
        x1=cx+int(math.cos(rad)*r1); y1=cy+int(math.sin(rad)*r1)
        x2=cx+int(math.cos(rad)*r2); y2=cy+int(math.sin(rad)*r2)
        cv2.line(img, (x1,y1), (x2,y2), col, max(1,lw-1))


RETICLE_DRAW = {
    'crosshair': _reticle_crosshair,
    'mil':       _reticle_mil,
    'bdc':       _reticle_bdc,
    'circle':    _reticle_circle,
    'tactical':  _reticle_tactical,
}

# ─── Шкала дальности ──────────────────────────────────────────────────────

def _draw_range_bar(img, H, W, dist, col):
    bx, by = int(W*0.22), H-28
    bw, bh = int(W*0.56), 10
    cv2.rectangle(img, (bx, by), (bx+bw, by+bh), (0,0,0), -1)
    frac = max(0.0, min(1.0, (dist-15)/(4200-15)))
    cv2.rectangle(img, (bx, by), (bx+int(bw*frac), by+bh), col, -1)
    cv2.rectangle(img, (bx, by), (bx+bw, by+bh), col, 1)
    # тики
    for d in range(500, 4200, 500):
        tx = bx + int(bw*(d-15)/(4200-15))
        cv2.line(img, (tx, by-4), (tx, by), col, 1)
        cv2.putText(img, f'{d}', (tx-12, by-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, col, 1, cv2.LINE_AA)
    cv2.putText(img, f'{dist} m', (bx+int(bw*frac)+4, by+9),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)


def _draw_hud(img, H, W, dist, target_type, col):
    tof = dist / 340.0
    az  = (dist * 0.017) % 360
    el  = math.degrees(math.atan2(1, dist/100))
    lines = [
        f'TGT : {TARGET_LABELS.get(target_type, target_type).upper()}',
        f'RNG : {dist} m',
        f'TOF : {tof:.2f} s',
        f'AZ  : {az:.1f}',
        f'EL  : {el:.1f}',
        f'MODE: LRF',
    ]
    for i, txt in enumerate(lines):
        y = 18 + i*18
        cv2.putText(img, txt, (9, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(img, txt, (9, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col,    1, cv2.LINE_AA)
    # правый блок
    ts = time.strftime('%H:%M:%S')
    for i, txt in enumerate([ts, 'ARMED', 'TRACKING']):
        y = 18 + i*18
        cv2.putText(img, txt, (W-95, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(img, txt, (W-95, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col,     1, cv2.LINE_AA)


def _draw_rings(img, cx, cy, R, col, alpha_frame):
    overlay = img.copy()
    for r_frac, thick in [(0.12, 1), (0.28, 1), (0.52, 1)]:
        cv2.circle(overlay, (cx, cy), int(R*r_frac), col, thick)
    # кардинальные тики
    for a in range(0, 360, 45):
        rad = math.radians(a)
        lw  = 2 if a % 90 == 0 else 1
        r1, r2 = int(R*0.53), int(R*0.58)
        x1=cx+int(math.cos(rad)*r1); y1=cy+int(math.sin(rad)*r1)
        x2=cx+int(math.cos(rad)*r2); y2=cy+int(math.sin(rad)*r2)
        cv2.line(overlay, (x1,y1), (x2,y2), col, lw)
    cv2.addWeighted(overlay, 0.7, img, 0.3, 0, img)


# ─── Главная функция ──────────────────────────────────────────────────────

class OverlayState:
    def __init__(self):
        self.target_type = 'infantry'
        self.reticle     = 'crosshair'
        self.color_name  = 'green'
        self.scale       = 1.0
        self.opacity     = 0.9
        self.brightness  = 1.0
        self.distance    = 500
        self.show_target = True
        self.show_rings  = True
        self.show_hud    = True
        self.show_bar    = True
        self.gray_mode   = False
        self.line_thickness = 1.5  # Добавляем параметр для толщины линий


def render(frame: np.ndarray, st: OverlayState) -> np.ndarray:
    """Нарисовать все элементы прицела на frame (in-place). Возвращает frame."""
    H, W = frame.shape[:2]
    cx, cy = W // 2, H // 2
    col = COLORS.get(st.color_name, (0, 255, 68))
    R   = int(min(W, H) * 0.28 * st.scale)
    lw  = max(1, int(st.line_thickness * st.scale))

    # Преобразование в черно-белый режим
    if st.gray_mode:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Преобразуем обратно в 3-канальный формат, чтобы совпадало с ожидаемым форматом
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    # Коррекция яркости
    if st.brightness != 1.0:
        frame = np.clip(frame.astype(np.float32) * st.brightness, 0, 255).astype(np.uint8)

    # Слой для прозрачного оверлея
    overlay = frame.copy()

    # 1. Силуэт цели
    if st.show_target:
        tgt_scale = st.scale * max(0.5, min(2.0, 1 - (st.distance - 15) / 8000))
        TARGET_DRAW[st.target_type](overlay, cx, cy, tgt_scale)

    # 2. Кольца масштаба
    if st.show_rings:
        _draw_rings(overlay, cx, cy, R, col, st.opacity)

    # 3. Прицел
    RETICLE_DRAW[st.reticle](overlay, cx, cy, R, col, lw)

    # Смешиваем слой с прицелом
    cv2.addWeighted(overlay, st.opacity, frame, 1 - st.opacity, 0, frame)

    # 4. HUD и шкала рисуем поверх без прозрачности
    if st.show_hud:
        _draw_hud(frame, H, W, st.distance, st.target_type, col)
    if st.show_bar:
        _draw_range_bar(frame, H, W, st.distance, col)

    return frame
