from controller import Robot, Emitter, Receiver
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())
name = robot.getName()

print(f"\n{'='*50}")
print(f"[{name}] MAVIC 2 PRO - OTONOM UCUS")
print(f"{'='*50}")

# === CİHAZLARI AL ===
camera = robot.getDevice("camera2")
display = robot.getDevice("display2")
emitter = robot.getDevice("emitter2")
receiver = robot.getDevice("receiver2")

# === MOTORLARI BUL ===
front_left_motor = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor = robot.getDevice("rear left propeller")
rear_right_motor = robot.getDevice("rear right propeller")

for m in [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]:
    if m:
        m.setPosition(float('inf'))
        m.setVelocity(0.0)

if camera: camera.enable(timestep)
if receiver: receiver.enable(timestep)

imu = robot.getDevice("inertial unit")
gps = robot.getDevice("gps")
gyro = robot.getDevice("gyro")

if imu: imu.enable(timestep)
if gps: gps.enable(timestep)
if gyro: gyro.enable(timestep)

# === ORİJİNAL C SABİTLERİ ===
K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P = 3.0
K_ROLL_P = 50.0
K_PITCH_P = 30.0

# ============================================
# === WAYPOINT AYARI ===
# ============================================
X_START = 0.0   # X başlangıç (batı)
X_END = 20.0     # X bitiş (doğu)
Y_START = 0.0    # Y başlangıç (kuzey)
Y_END = 8.0  # Y bitiş (güney)
STEP = 2.0        # Şerit genişliği (X'te artış)

# Z = Yükseklik (dikey eksen)
TARGET_ALTITUDE = 8.0

print(f"[{name}] ========================================")
print(f"[{name}] WAYPOINT SISTEMI:")
print(f"[{name}]   X ekseni (batı-doğu): {X_START} -> {X_END}")
print(f"[{name}]   Y ekseni (kuzey-güney): {Y_START} -> {Y_END}")
print(f"[{name}]   Z ekseni (yükseklik): {TARGET_ALTITUDE}m")
print(f"[{name}] ========================================")

# === WAYPOINT OLUŞTUR (LAWN-MOWER / ÇİM BİÇME) ===
# Mantık: X sabit kalır, Y ileri-geri gider
# Sonra X artar, Y tekrar ileri-geri
# Desen:  (28,24)->(28,32)->(30,32)->(30,24)->(32,24)->...

waypoints = []
x = X_START
ileri = True  # İlk sefer Y_START'dan Y_END'e git (ileri)

while x <= X_END:
    y1, y2 = Y_START, Y_END
    
    if not ileri:
        # Geri dönüş: Y_END'den Y_START'a
        y1, y2 = y2, y1
    
    # Bu X konumunda iki nokta: biri başlangıç, biri bitiş
    wp1 = (x, y1)
    wp2 = (x, y2)
    waypoints.append(wp1)
    waypoints.append(wp2)
    
    print(f"[{name}]   Sütun X={x}: {wp1} -> {wp2}")
    
    x += STEP      # Sonraki sütuna geç (X artar)
    ileri = not ileri  # Yönü ters çevir

print(f"[{name}] ========================================")
print(f"[{name}] Toplam {len(waypoints)} waypoint olusturuldu.")
print(f"[{name}] Ornek: WP0={waypoints[0]}, WP1={waypoints[1]}, WP2={waypoints[2]}")
print(f"[{name}] ========================================")

# === BAŞLANGIÇ POZİSYONU KAYDET ===
start_x, start_y = 0.0, 0.0
if gps:
    pos = gps.getValues()
    start_x = pos[0]
    start_y = pos[1]
    print(f"[{name}] Baslangic pozisyonu: X={start_x:.2f}, Y={start_y:.2f}, Z={pos[2]:.2f}")

# ============================================
# === 1. KALKIŞ (Sadece Z artsın, X-Y sabit) ===
# ============================================
print(f"[{name}] >>> KALKIS BASLIYOR... (Sadece Z, X-Y sabit)")

for i in range(50):
    thrust = K_VERTICAL_THRUST * (i / 50.0)
    
    # SADECE DÜZ THRUST - HİÇ STABİLİZASYON YOK!
    # X ve Y'yi değiştirmemek için roll=pitch=0 varsayımıyla düz kalkış
    front_left_motor.setVelocity(thrust)
    front_right_motor.setVelocity(-thrust)
    rear_left_motor.setVelocity(-thrust)
    rear_right_motor.setVelocity(thrust)
    
    robot.step(timestep)

# ============================================
# === 2. SADECE YÜKSELME (Z=10 olana kadar) ===
# ============================================
print(f"[{name}] >>> YUKSELME: Z={TARGET_ALTITUDE}m olana kadar...")

while robot.step(timestep) != -1:
    if gps:
        pz = gps.getValues()[2]
    else:
        pz = 0.0
    
    if imu:
        rpy = imu.getRollPitchYaw()
        roll = rpy[0]
        pitch = rpy[1]
    else:
        roll = pitch = 0
    
    if gyro:
        rv = gyro.getValues()[0]
        pv = gyro.getValues()[1]
    else:
        rv = pv = 0
    
    # SADECE DİKEY STABİLİZASYON
    # Yatay disturbance = 0 (X ve Y sabit kalsın!)
    roll_input = K_ROLL_P * max(-1.0, min(1.0, roll)) + rv
    pitch_input = K_PITCH_P * max(-1.0, min(1.0, pitch)) + pv
    
    altitude_error = TARGET_ALTITUDE - pz + K_VERTICAL_OFFSET
    clamped = max(-1.0, min(1.0, altitude_error))
    vertical_input = K_VERTICAL_P * (clamped ** 3.0)
    
    # YATAY HAREKET YOK! roll_disturbance=0, pitch_disturbance=0
    fl = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input
    fr = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input
    rl = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input
    rr = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input
    
    front_left_motor.setVelocity(fl)
    front_right_motor.setVelocity(-fr)
    rear_left_motor.setVelocity(-rl)
    rear_right_motor.setVelocity(rr)
    
    # Hedef yüksekliğe ulaştık mı?
    if abs(TARGET_ALTITUDE - pz) < 0.5:
        print(f"[{name}] >>> Hedef yukseklige ulasildi! Z={pz:.2f}")
        break

# ============================================
# === 3. ANA DÖNGÜ (Waypoint takibi) ===
# ============================================
wp_index = 0
frame_count = 0
kutu_bulundu = False
THRESHOLD = 1.5

print(f"[{name}] >>> YATAY HAREKET BASLIYOR...")

while robot.step(timestep) != -1:
    # === SENSÖR OKU ===
    if imu:
        rpy = imu.getRollPitchYaw()
        roll = rpy[0]
        pitch = rpy[1]
    else:
        roll = pitch = 0
    
    if gps:
        pos = gps.getValues()
        px = pos[0]   # X = yatay
        py = pos[1]   # Y = yatay
        pz = pos[2]   # Z = yükseklik
    else:
        px = py = pz = 0
    
    if gyro:
        rv = gyro.getValues()[0]
        pv = gyro.getValues()[1]
    else:
        rv = pv = 0
    
    # === HEDEF KONTROL ===
    if wp_index >= len(waypoints):
        print(f"[{name}] >>>>> TARAMA BITTI!")
        # İniş
        for i in range(100):
            t = K_VERTICAL_THRUST * (1.0 - i/100.0)
            front_left_motor.setVelocity(t)
            front_right_motor.setVelocity(-t)
            rear_left_motor.setVelocity(-t)
            rear_right_motor.setVelocity(t)
            robot.step(timestep)
        break
    
    # Hedef waypoint
    tx, ty = waypoints[wp_index]
    
    # Farkları hesapla
    dx = tx - px
    dy_w = ty - py
    dz = TARGET_ALTITUDE - pz
    
    dist_xy = math.sqrt(dx*dx + dy_w*dy_w)
    
    # Hedefe vardık mı?
    if dist_xy < THRESHOLD and abs(dz) < 0.5:
        wp_index += 1
        print(f"[{name}] >>> WP {wp_index}/{len(waypoints)} | X={px:.1f} Y={py:.1f} Z={pz:.1f}")
        if emitter:
            emitter.send(f"PROGRESS:{name}:{wp_index}/{len(waypoints)}")
    
    # === STABILIZASYON ===
    roll_input = K_ROLL_P * max(-1.0, min(1.0, roll)) + rv
    pitch_input = K_PITCH_P * max(-1.0, min(1.0, pitch)) + pv
    
    # Yükseklik kontrolü (Z)
    altitude_error = TARGET_ALTITUDE - pz + K_VERTICAL_OFFSET
    clamped = max(-1.0, min(1.0, altitude_error))
    vertical_input = K_VERTICAL_P * (clamped ** 3.0)
    
    # === YATAY HAREKET (Çok düşük kazanç) ===
    # X sınır kontrolü: X_START ile X_END arasında kal
    # Y sınır kontrolü: Y_START ile Y_END arasında kal
    
    roll_disturbance = 0.0
    pitch_disturbance = 0.0
    
    # Sadece hedefe yakınsak hareket et (uzaktaysa yavaşça)
    if abs(dx) > 0.3:
        pitch_disturbance = dx * 1.0 # X yönünde (çok düşük)
    if abs(dy_w) > 0.3:
        roll_disturbance = -dy_w * 0.25 # Y yönünde (çok düşük)
    
    # SINIR KONTROLÜ: X ve Y sınırların dışına çıkmasın!
    if px < X_START - 1.0:
        pitch_disturbance += 2.0  # Sağa it (X artır)
        print(f"[{name}] UYARI: X sinir disi ({px:.1f}), duzeltme!")
    if px > X_END + 1.0:
        pitch_disturbance -= 2.0  # Sola it (X azalt)
        print(f"[{name}] UYARI: X sinir disi ({px:.1f}), duzeltme!")
    
    roll_total = roll_input + roll_disturbance
    pitch_total = pitch_input + pitch_disturbance
    yaw_total = 0.0
    
    # === MOTORLAR ===
    fl = K_VERTICAL_THRUST + vertical_input - roll_total + pitch_total - yaw_total
    fr = K_VERTICAL_THRUST + vertical_input + roll_total + pitch_total + yaw_total
    rl = K_VERTICAL_THRUST + vertical_input - roll_total - pitch_total + yaw_total
    rr = K_VERTICAL_THRUST + vertical_input + roll_total - pitch_total - yaw_total
    
    front_left_motor.setVelocity(fl)
    front_right_motor.setVelocity(-fr)
    rear_left_motor.setVelocity(-rl)
    rear_right_motor.setVelocity(rr)
    
    # === KAMERA + TESPIT ===
    if camera and display:
        frame_count += 1
        image = camera.getImage()
        
        if image and frame_count % 5 == 0:
            w = camera.getWidth()
            h = camera.getHeight()
            
            for py_cam in range(0, h, 8):
                for px_cam in range(0, w, 8):
                    r = camera.imageGetRed(image, w, px_cam, py_cam)
                    g = camera.imageGetGreen(image, w, px_cam, py_cam)
                    b = camera.imageGetBlue(image, w, px_cam, py_cam)
                    color = (r << 16) | (g << 8) | b
                    display.setColor(color)
                    display.drawPixel(px_cam, py_cam)
            
            if not kutu_bulundu and frame_count % 20 == 0:
                kirmizi = 0
                top_x = 0
                top_y = 0
                
                for py_cam in range(0, h, 4):
                    for px_cam in range(0, w, 4):
                        r = camera.imageGetRed(image, w, px_cam, py_cam)
                        g = camera.imageGetGreen(image, w, px_cam, py_cam)
                        b = camera.imageGetBlue(image, w, px_cam, py_cam)
                        
                        if r > 150 and g < 70 and b < 70:
                            kirmizi += 1
                            top_x += px_cam
                            top_y += py_cam
                
                if kirmizi > 25:
                    ox = top_x / kirmizi
                    oy = top_y / kirmizi
                    
                    display.setColor(0x00FF00)
                    display.drawRectangle(int(ox)-25, int(oy)-25, 50, 50)
                    display.drawText("KUTU!", int(ox)-30, int(oy)-35)
                    
                    gercek_x = px + (ox - w/2) * 0.006
                    gercek_y = py + (oy - h/2) * 0.009
                    
                    print(f"[{name}] *** KUTU! X={gercek_x:.2f} Y={gercek_y:.2f}")
                    if emitter:
                        emitter.send(f"ALERT:{name}:KUTU:X={gercek_x:.2f}:Y={gercek_y:.2f}")
                    kutu_bulundu = True
    
    if receiver:
        while receiver.getQueueLength() > 0:
            data = receiver.getString()
            if name not in data:
                print(f"[{name}] <<< {data}")
            receiver.nextPacket()