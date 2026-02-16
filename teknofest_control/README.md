# TEKNOFEST Savaşan İHA - GPS Takip Sistemi

## 📋 İçindekiler
1. [Genel Bakış](#genel-bakış)
2. [Dosya Yapısı](#dosya-yapısı)
3. [Hızlı Başlangıç](#hızlı-başlangıç)
4. [Adım Adım Kullanım](#adım-adım-kullanım)
5. [Parametre Ayarları](#parametre-ayarları)
6. [ROS2 Topic'leri](#ros2-topicleri)
7. [Sorun Giderme](#sorun-giderme)
8. [Algoritmalar](#algoritmalar)

---

## 🎯 Genel Bakış

Bu sistem, birden fazla hedef İHA arasından en uygun olanı seçip L1 Guidance algoritması ile takip eder. Amaç, hedefi kamera görüş açısına almaktır.

### Temel Özellikler
- **Çoklu Hedef Seçimi**: 5 farklı kriter ile en uygun hedefi belirler
- **L1 Guidance**: Sabit kanatlı uçak için optimize edilmiş takip algoritması
- **Kamera FOV Optimizasyonu**: Hedefin kameraya girmesi için açı hesabı
- **Hysteresis**: Gereksiz hedef değişimlerini önler

---

## 📁 Dosya Yapısı

```
teknofest_ws/
├── src/
│   ├── teknofest_control/           # ANA KONTROL PAKETİ
│   │   ├── teknofest_control/
│   │   │   ├── gps_tracking_node.py # ⭐ Ana takip node'u
│   │   │   └── l1_guidance.py       # L1 algoritması (ROS2 bağımsız)
│   │   ├── config/
│   │   │   └── tracking_params.yaml # ⭐ TÜM PARAMETRELER BURADA
│   │   └── launch/
│   │       └── tracking.launch.py   # Launch dosyası (opsiyonel)
│   │
│   ├── teknofest_simulation/        # SİMÜLASYON
│   │   ├── launch/
│   │   │   └── start_multi_aircraft.sh  # ⭐ Simülasyonu başlatır
│   │   ├── models/
│   │   │   └── hero_cessna/         # Kameralı uçak modeli
│   │   └── worlds/
│   │       └── competition.sdf      # Gazebo dünyası
│   │
│   └── teknofest_vision/            # KAMERA
│       └── launch/
│           └── start_camera_bridge.sh  # ⭐ Kamera bridge scripti
```

### ⭐ Önemli Dosyalar
| Dosya | Ne İşe Yarar |
|-------|--------------|
| `gps_tracking_node.py` | Ana takip algoritması - PX4 ile haberleşir |
| `l1_guidance.py` | L1 algoritması + hedef seçimi (saf Python) |
| `tracking_params.yaml` | **TÜM ayarlar bu dosyada** |
| `start_multi_aircraft.sh` | 3 uçaklı simülasyonu başlatır |
| `start_camera_bridge.sh` | Gazebo kamerasını ROS2'ye aktarır |

---

## 🚀 Hızlı Başlangıç

### Ön Gereksinimler
```bash
# ROS2 Humble kurulu olmalı
# PX4-Autopilot kurulu olmalı (~/Desktop/PX4-Autopilot)
# Gazebo Harmonic kurulu olmalı
```

### 3 Adımda Çalıştır

```bash
# ADIM 1: Simülasyonu başlat (Terminal 1)
cd ~/teknofest_ws/src/teknofest_simulation/launch
./start_multi_aircraft.sh

# ADIM 2: Tracking node başlat (Terminal 2)
source ~/teknofest_ws/install/setup.bash
ros2 run teknofest_control gps_tracking_node --ros-args \
    -p namespaces.targets:='/px4_1,/px4_2'

# ADIM 3: (Opsiyonel) Kamera görüntüsü (Terminal 3)
cd ~/teknofest_ws/src/teknofest_vision/launch
./start_camera_bridge.sh
```

---

## 📖 Adım Adım Kullanım

### ADIM 1: Simülasyonu Başlat

```bash
# Terminal 1
cd ~/teknofest_ws/src/teknofest_simulation/launch
./start_multi_aircraft.sh
```

**Beklenen Çıktı:**
```
═══════════════════════════════════════════════════════════════
✓ TEKNOFEST Multi-Aircraft Simülasyonu Hazır!
═══════════════════════════════════════════════════════════════

Spawn Edilen Modeller:
  • hero_cessna
  • rc_cessna (ENEMY1)
  • rc_cessna (ENEMY2)

QGroundControl Bağlantıları:
  Uçak 1 (HERO):   udp://127.0.0.1:14540
  Uçak 2 (ENEMY1): udp://127.0.0.1:14541
  Uçak 3 (ENEMY2): udp://127.0.0.1:14542
```

**⏳ 30-60 saniye bekleyin** - Tüm modellerin yüklenmesi zaman alır.

---

### ADIM 2: QGroundControl'da Uçakları Kaldır

1. QGroundControl'u açın
2. Her uçak için ayrı ayrı:
   - Vehicle seçin (sağ üstten)
   - **Arm** (güç düğmesi)
   - **Takeoff** veya mission başlatın
3. Uçaklar havada olduğunda ADIM 3'e geçin

---

### ADIM 3: Tracking Node'u Başlat

```bash
# Terminal 2
source ~/teknofest_ws/install/setup.bash

# TEK HEDEF TAKİBİ (sadece px4_1)
ros2 run teknofest_control gps_tracking_node --ros-args \
    -p namespaces.targets:='/px4_1'

# VEYA ÇOKLU HEDEF (px4_1 ve px4_2 - en iyi hedef otomatik seçilir)
ros2 run teknofest_control gps_tracking_node --ros-args \
    -p namespaces.targets:='/px4_1,/px4_2'
```

**Parametre Açıklamaları:**
| Parametre | Değer | Açıklama |
|-----------|-------|----------|
| `namespaces.targets` | `/px4_1,/px4_2` | Hedef uçak namespace'leri (virgülle ayrılmış) |

**Otomatik ID Atama:** Her namespace'in ID'si otomatik çıkarılır: `/px4_1` → ID 1, `/px4_2` → ID 2

**Multi-Vehicle Namespace'ler:**
- `/fmu/...` veya boş = İlk uçak (HERO, instance 0)
- `/px4_1/fmu/...` = İkinci uçak (ENEMY1, instance 1)
- `/px4_2/fmu/...` = Üçüncü uçak (ENEMY2, instance 2)

---

### ADIM 4: (Opsiyonel) Kamera Görüntüsü

```bash
# Terminal 3
cd ~/teknofest_ws/src/teknofest_vision/launch
./start_camera_bridge.sh

# Görüntüyü izlemek için (yeni terminal)
ros2 run rqt_image_view rqt_image_view /hero/camera/image_raw
```

---

## ⚙️ Parametre Ayarları

**Tüm parametreler bu dosyada:**
```
~/teknofest_ws/src/teknofest_control/config/tracking_params.yaml
```

### Hedef Seçim Ağırlıkları

```yaml
weights:
  distance: 0.25        # Yakın hedefler tercih edilir
  angle: 0.30           # Kuyruk pozisyonu bonus alır
  speed: 0.15           # Yavaş hedefler tercih edilir
  camera_fov: 0.20      # Kamera FOV'a uygun hedefler
  reachability: 0.10    # Yakalama süresi makul olanlar
```

**Toplam = 1.0 olmalı!**

### Kamera FOV Ayarları

```yaml
camera_fov:
  horizontal_fov: 80.0    # Yatay görüş açısı (derece)
  vertical_fov: 60.0      # Dikey görüş açısı (derece)
  optimal_distance: 50.0  # En iyi görüntüleme mesafesi
  max_distance: 200.0     # Maksimum görüntüleme mesafesi
```

### L1 Guidance Ayarları

```yaml
l1_guidance:
  l1_distance: 50.0       # L1 referans mesafesi (metre)
  l1_damping: 0.85        # Sönümleme (0.7-0.9)
  adaptive_l1: true       # Hıza göre adaptif L1
```

### Hysteresis (Hedef Tutunma)

```yaml
target_selection:
  current_target_bonus: 1.5       # Mevcut hedefe bonus (1.5x)
  min_score_diff_to_switch: 0.3   # Hedef değiştirme eşiği
```

---

## 📡 ROS2 Topic'leri

### Yayınlanan Topic'ler (Publisher)

```bash
# Durum bilgisi
ros2 topic echo /tracking/state

# Hedef skorları (JSON)
ros2 topic echo /tracking/target_scores

# En iyi hedef bilgisi
ros2 topic echo /tracking/best_target

# Kamera FOV durumu
ros2 topic echo /tracking/camera_fov_status

# Sanal hedef noktası (L1 point)
ros2 topic echo /tracking/virtual_target
```

### Dinlenen Topic'ler (Subscriber)

```bash
# PX4 pozisyon bilgisi
/fmu/out/vehicle_local_position
/fmu/out/vehicle_global_position

# Hedef uçak pozisyonu
/px4_1/fmu/out/vehicle_local_position
/px4_1/fmu/out/vehicle_global_position
```

### Faydalı Komutlar

```bash
# Tüm tracking topic'lerini listele
ros2 topic list | grep tracking

# Hedef skorlarını izle
ros2 topic echo /tracking/target_scores

# Topic frekansını kontrol et
ros2 topic hz /fmu/out/vehicle_local_position
```

---

## 🔧 Sorun Giderme

### Kamera Görüntüsü Gelmiyor

**1. Gazebo topic'lerini kontrol et:**
```bash
gz topic -l | grep image
```

**Beklenen çıktı:**
```
/world/competition/model/hero_cessna/link/base_link/sensor/front_camera/image
```

**2. Model ismini doğrula:**
```bash
gz model --list
```

**3. Manuel bridge başlat:**
```bash
# Eğer model ismi farklıysa (örn: hero_cessna_0)
cd ~/teknofest_ws/src/teknofest_vision/launch
./start_camera_bridge.sh hero_cessna_0
```

---

### Tracking Node Hedef Görmüyor

**1. PX4 topic'lerini kontrol et:**
```bash
ros2 topic list | grep px4_1
```

**Beklenen:**
```
/px4_1/fmu/out/vehicle_local_position
/px4_1/fmu/out/vehicle_global_position
```

**2. Topic verisi geliyor mu?**
```bash
ros2 topic echo /px4_1/fmu/out/vehicle_local_position --once
```

**3. Namespace doğru mu kontrol et:**
```bash
# Doğru format (çoklu hedef)
ros2 run teknofest_control gps_tracking_node --ros-args \
    -p namespaces.targets:='/px4_1,/px4_2'
```

---

### Offboard Mod Aktif Olmuyor

**1. Yeterli setpoint gönderildi mi?**
PX4, offboard moda geçmeden önce en az 100 setpoint bekler (~2 saniye).

**2. QGC'de mod kontrolü:**
- Flight mode'un "Offboard" olduğunu doğrulayın
- Arm durumunu kontrol edin

**3. Log'ları incele:**
```bash
# Node çıktısında şunu arayın:
# "✓ Offboard mod aktif!"
```

---

### Simülasyon Başlamıyor

**1. PX4 build kontrol:**
```bash
cd ~/Desktop/PX4-Autopilot
make px4_sitl gz_rc_cessna
```

**2. Eski işlemleri temizle:**
```bash
pkill -9 px4
pkill -9 -f "gz sim"
```

**3. Tekrar başlat:**
```bash
cd ~/teknofest_ws/src/teknofest_simulation/launch
./start_multi_aircraft.sh
```

---

## 📚 Algoritmalar

### Hedef Seçim Algoritması

**5 Kriter ile Puanlama:**

1. **Mesafe Skoru (25%)**: Yakın hedefler yüksek puan alır
2. **Açı Skoru (30%)**: Kuyruk pozisyonundaki hedefler bonus alır
3. **Hız Skoru (15%)**: Yavaş (yakalanabilir) hedefler tercih edilir
4. **Kamera FOV Skoru (20%)**: Görüş açısına uygun hedefler tercih edilir
5. **Ulaşılabilirlik Skoru (10%)**: Yakalama süresi makul olanlar tercih edilir

**Formül:**
```
Score = (0.25 × Distance) + (0.30 × Angle) + (0.15 × Speed) 
      + (0.20 × CameraFOV) + (0.10 × Reachability)
```

### L1 Guidance Algoritması

L1 algoritması, hedefin arkasında sanal bir nokta (L1 point) hesaplar ve uçağı bu noktaya yönlendirir.

```
        Hedef (Target)
           ↓
    ═══════●═══════► Hedef yönü
           │
           │ L1 Distance
           │
           ●←── L1 Point (Sanal Hedef)
          ╱
         ╱
        ╱ Uçak bu noktaya yönelir
       ╱
      ●
    Uçak
```

**Avantajları:**
- Pürüzsüz takip yörüngesi
- Sabit kanatlı uçaklar için uygun
- Ani manevraları önler

---

## 📞 Hızlı Referans

### En Sık Kullanılan Komutlar

```bash
# Simülasyonu başlat
cd ~/teknofest_ws/src/teknofest_simulation/launch && ./start_multi_aircraft.sh

# Tracking başlat
source ~/teknofest_ws/install/setup.bash
ros2 run teknofest_control gps_tracking_node --ros-args -p namespaces.targets:='/px4_1,/px4_2'

# Kamera bridge
cd ~/teknofest_ws/src/teknofest_vision/launch && ./start_camera_bridge.sh

# Kamera görüntüsü
ros2 run rqt_image_view rqt_image_view /hero/camera/image_raw

# Debug - hedef skorları
ros2 topic echo /tracking/target_scores

# Tüm topic'ler
ros2 topic list | grep -E "tracking|fmu|px4"
```

### Parametre Dosyası Yolu
```
~/teknofest_ws/src/teknofest_control/config/tracking_params.yaml
```

### Build Komutu
```bash
cd ~/teknofest_ws
colcon build --packages-select teknofest_control --symlink-install
source install/setup.bash
```

---

## 📝 Notlar

- Simülasyon başladıktan sonra **30-60 saniye bekleyin**
- Uçaklar **havada** olmalı (takeoff yapılmış)
- Offboard mod için uçak **arm** edilmiş olmalı
- Parametre değişikliklerinden sonra **node'u yeniden başlatın**

---

*HAVK Takımı - TEKNOFEST 2026*
