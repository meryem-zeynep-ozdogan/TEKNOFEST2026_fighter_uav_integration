# TEKNOFEST Savaşan İHA - GPS Takip Sistemi

## 📋 Genel Bakış

Bu paket, TEKNOFEST Savaşan İHA yarışması için geliştirilmiş **L1 Adaptive Guidance** tabanlı GPS takip sistemini içerir.

### Temel Özellikler

- ✅ **Weighted Scoring** ile akıllı hedef seçimi
- ✅ **L1 Guidance** algoritması ile smooth takip
- ✅ **Rüzgar kompanzasyonu** (Crab Angle hesabı)
- ✅ **Rate Limiting** ile ani manevralar önleme
- ✅ **Loiter modu** (çok yaklaşıldığında etrafında dönme)
- ✅ **20+ uçak** senaryolarında yüksek performans
- ✅ **4 saniye kesintisiz** kilitlenme hedefi

## 🏗️ Mimari

```
teknofest_control/
├── config/
│   └── tracking_params.yaml      # Tüm parametreler
├── launch/
│   └── tracking.launch.py        # Launch dosyası
├── teknofest_control/
│   ├── __init__.py
│   ├── gps_tracking_node.py      # Ana takip node'u
│   ├── guidance_algorithms.py    # L1 Guidance modülü
│   ├── smooth_control.py         # Filtre ve rate limiterlar
├── package.xml
├── setup.py
└── README.md
```

## 🚀 Kurulum

### Gereksinimler

- ROS2 Humble veya Foxy
- Python 3.8+
- PX4 Autopilot (MicroXRCE-DDS agent)
- px4_msgs paketi

### Build

```bash
cd ~/teknofest_ws
colcon build --packages-select teknofest_control
source install/setup.bash
```

## 📖 Kullanım

### Gerçek Uçuş / Gazebo Simülasyonu

```bash
# PX4 SITL veya gerçek uçuş için
ros2 launch teknofest_control tracking.launch.py
```

### Test (Simüle Hedeflerle)

```bash
# Sahte hedeflerle test
ros2 launch teknofest_control tracking.launch.py use_simulator:=true num_targets:=10
```

### Parametreleri Değiştirme

```bash
# Özel parametre dosyası ile
ros2 launch teknofest_control tracking.launch.py config_file:=/path/to/custom_params.yaml
```

## ⚙️ Parametreler

### Hedef Seçimi Ağırlıkları

| Parametre | Varsayılan | Açıklama |
|-----------|------------|----------|
| `weights.distance` | 0.35 | Mesafe ağırlığı (yakın = iyi) |
| `weights.angle` | 0.45 | Açı uygunluğu (kuyruk = çok iyi) |
| `weights.speed` | 0.20 | Hız faktörü (yavaş = iyi) |

### L1 Guidance

| Parametre | Varsayılan | Açıklama |
|-----------|------------|----------|
| `l1_guidance.l1_distance` | 50.0 m | Hedefin gerisinde kalma mesafesi |
| `l1_guidance.l1_damping` | 0.85 | Sönümleme faktörü |
| `l1_guidance.adaptive_l1` | true | Hıza göre adaptif L1 |

### Smooth Kontrol

| Parametre | Varsayılan | Açıklama |
|-----------|------------|----------|
| `smoothing.heading_rate_limit` | 15.0 °/s | Maks heading değişim hızı |
| `smoothing.command_smoothing_alpha` | 0.15 | EMA smoothing faktörü |

## 📊 Algoritma Detayları

### 1. Hedef Seçimi (Weighted Scoring)

```
Score = (W1 × 1/Mesafe) + (W2 × Açı_Faktörü) + (W3 × Hız_Faktörü)
```

**Açı Faktörü Hesabı:**
- Kuyruk pozisyonunda (arkada): **+2.5x bonus**
- Kafa kafaya (tehlikeli): **0.2x ceza**
- Yan açıda: Lineer skor

### 2. L1 Guidance Mantığı

```
┌────────────────────────────────────────────────────┐
│                    HEDEF                           │
│                      ●──────→ (hız vektörü)       │
│                      │                             │
│                      │ L1 mesafesi (50m)          │
│                      │                             │
│                      ▼                             │
│               [SANAL HEDEF]                        │
│                      ▲                             │
│                      │                             │
│              LOS vektörü                           │
│                      │                             │
│                      │                             │
│                  BİZ ●                             │
└────────────────────────────────────────────────────┘
```

**L1 Lateral İvme:**
```
a_cmd = 2 × V² / L1 × sin(η)
```
Burada η = LOS açısı - mevcut heading

### 3. Smooth Kontrol Pipeline

```
Ham Komut → Outlier Rejection → LPF → Rate Limiter → Deadband → Final Komut
```

## 🎯 Durum Makinesi

```
┌─────────────┐     hedef var     ┌──────────────┐
│    IDLE     │ ─────────────────→│  SEARCHING   │
└─────────────┘                   └──────────────┘
                                         │
                                   d < 150m
                                         ▼
                                  ┌──────────────┐
                                  │ APPROACHING  │
                                  └──────────────┘
                                         │
                                   d < 80m
                                         ▼
                                  ┌──────────────┐
                                  │  PURSUING    │
                                  └──────────────┘
                                         │
                                  stabil takip
                                         ▼
                                  ┌──────────────┐
                                  │   LOCKED     │◄── 4 saniye = ✓
                                  └──────────────┘
                                         │
                                   d < 25m
                                         ▼
                                  ┌──────────────┐
                                  │  LOITERING   │
                                  └──────────────┘
```

## 🔧 Sorun Giderme

### "Uçak ani manevralar yapıyor"

1. `smoothing.heading_rate_limit` değerini düşür (10-12 °/s)
2. `smoothing.command_smoothing_alpha` değerini düşür (0.08-0.12)
3. `l1_guidance.l1_distance` değerini artır (60-80m)

### "Hedefi kaybediyor"

1. `state_machine.lock_distance` değerini artır (100m)
2. `weights.distance` ağırlığını artır (0.45)
3. `l1_guidance.l1_damping` değerini düşür (0.75)

### "Kilitlenme süresi tutmuyor"

1. Daha agresif takip için `smoothing.heading_rate_limit` artır (18-20 °/s)
2. `l1_guidance.l1_distance` değerini azalt (40m)

## 📝 Topic'ler

### Subscriber'lar

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/fmu/out/vehicle_local_position` | VehicleLocalPosition | PX4 lokal pozisyon |
| `/fmu/out/vehicle_global_position` | VehicleGlobalPosition | PX4 GPS pozisyon |
| `/competition/target_list` | Float64MultiArray | Hedef listesi |

### Publisher'lar

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/fmu/in/trajectory_setpoint` | TrajectorySetpoint | PX4 hız komutu |
| `/fmu/in/offboard_control_mode` | OffboardControlMode | Offboard heartbeat |
| `/tracking/state` | String | Mevcut durum |
| `/tracking/virtual_target` | PoseStamped | Sanal hedef (viz) |

## 🧪 Test

```bash
# Unit testler
cd ~/teknofest_ws
colcon test --packages-select teknofest_control

# Simülasyon testi
ros2 run teknofest_control target_simulator &
ros2 run teknofest_control gps_tracking_node
```

## 📚 Referanslar

1. Park, S., Deyst, J., & How, J. P. (2007). "Performance and Lyapunov Stability of a Nonlinear Path-Following Guidance Method"
2. Beard, R. W., & McLain, T. W. (2012). "Small Unmanned Aircraft: Theory and Practice"
3. PX4 Offboard Control: https://docs.px4.io/main/en/flight_modes/offboard.html

## 📄 Lisans

Apache-2.0

## 👥 Katkıda Bulunanlar

HAVK Takımı - TEKNOFEST 2026
