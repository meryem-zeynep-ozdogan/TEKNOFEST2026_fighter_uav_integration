# TEKNOFEST 2026 – Savaşan İHA Entegrasyon Çalışma Alanı

Bu repo; **PX4 SITL + Gazebo (Harmonic) simülasyonu**, **ROS 2 kontrol (L1 Guidance tabanlı hedef takibi)**, **kamera bridge** ve **Mock yarışma sunucusu (WS + HTTP)** bileşenlerini tek bir çalışma alanında bir araya getirir.

> Not: Repo bir **colcon workspace** gibi kullanılıyor (kök dizinde `build/`, `install/`, `log/` klasörleri mevcut). Paketler doğrudan kök dizin altında yer alıyor.

## İçindekiler
- [Bileşenler](#bileşenler)
- [Hızlı Başlangıç](#hızlı-başlangıç)
  - [A) PX4 SITL + Gazebo + Tracking](#a-px4-sitl--gazebo--tracking)
  - [B) Mock Sunucu + Tracking (PX4 olmadan)](#b-mock-sunucu--tracking-px4-olmadan)
  - [C) PX4 → Mock → Tracking (karma test)](#c-px4--mock--tracking-karma-test)
- [Mimari / Veri Akışı](#mimari--veri-akışı)
- [ROS 2 Topic / Arayüz Referansı](#ros-2-topic--arayüz-referansı)
- [Konfigürasyon](#konfigürasyon)
- [Mock Sunucu ve Dashboard](#mock-sunucu-ve-dashboard)
- [Sorun Giderme](#sorun-giderme)

---

## Bileşenler

### 1) `teknofest_control` (ROS 2 – Kontrol ve Takip)
ROS 2 `ament_python` paketi.

Öne çıkan node’lar (entry points):
- `gps_tracking_node`: PX4 telemetri + hedef listesi alır, **L1 Guidance + hedef seçimi** ile setpoint üretir, PX4’e Offboard setpoint basar.
- `mock_target_receiver`: Mock sunucudan (Socket.IO) telemetri alır ve `gps_tracking_node`’un beklediği topic’lere yayınlar.
- `px4_to_mock_bridge`: PX4 SITL’deki rakip instance’ların GPS/hız/yaw verisini mock sunucuya iletir.
- `mock_server_bridge`: Mock sunucudan gelen telemetriyi ROS topic’lerine yayınlayan alternatif/legacy köprü.

Algoritma tarafı:
- `teknofest_control/teknofest_control/l1_guidance.py`: ROS’tan bağımsız, saf Python + NumPy ile L1 guidance, hedef seçimi ve takip durum makinesi.

Konfigürasyon:
- `teknofest_control/config/tracking_params.yaml`: `gps_tracking_node` parametreleri.
- `teknofest_control/config/mock_tracking_params.yaml`: mock senaryo parametreleri (bazı bölümler legacy olabilir; bkz. [Bilinen noktalar](#bilinen-noktalar)).

Launch:
- `teknofest_control/launch/tracking.launch.py`
- `teknofest_control/launch/mock_tracking.launch.py`

### 2) `teknofest_simulation` (PX4 SITL + Gazebo Harmonic)
ROS paketi değil; Gazebo world/model ve başlatma script’i içerir.

- `teknofest_simulation/worlds/competition.sdf`: Dünya, **Şanlıurfa GAP Havalimanı** spherical coordinates ile ayarlı.
- `teknofest_simulation/launch/start_multi_aircraft.sh`: 3 uçak (HERO + 2 rakip) için PX4 SITL + Gazebo başlatır.

### 3) `teknofest_vision` (Kamera bridge)
- `teknofest_vision/launch/start_camera_bridge.sh`: Gazebo image topic’ini ROS 2 `image_raw` topic’ine bridge eder.
- `teknofest_vision/launch/camera_bridge.launch.py`: aynı işi launch ile yapar.

### 4) `mock/` (Mock yarışma sunucusu + senaryolar + dashboard)
- `mock/server/ws_server.py`: Flask + Flask-SocketIO (gevent) tabanlı mock sunucu.
  - WS event’leri: `start_multiple`, `telemetry`, `external_telemetry`, `lock_attempt`, `lock_response`, `lock_success`.
  - HTTP endpoint’leri: `/api/giris`, `/api/sunucusaati`, `/api/kamikaze_bilgisi`, `/api/kilitlenme_bilgisi`, `/api/qr`, `/api/takeoff`.
- `mock/scenarios/*.py`: basit senaryo jeneratörleri (`hss_approach`, `circular`, `straight`).
- `mock/config/hss_zones.json`: HSS bölgeleri (şu an dairesel alan).
- `mock/dashboard/*`: Leaflet tabanlı canlı telemetri haritası.

---

## Hızlı Başlangıç

### Ön koşullar
Bu repo, ortamınıza göre farklı bağımlılıklar ister:
- ROS 2 (tercihen **Humble**)
- `colcon` ve Python build araçları
- PX4-Autopilot (SITL) + Gazebo Harmonic
- QGroundControl (SITL araçlarını arm/takeoff için)
- Mock sunucu için Python paketleri: `flask`, `flask_socketio`, `gevent`, `python-socketio`

> Sistem paket isimleri dağıtıma göre değişebilir. Eğer build veya runtime’da import hatası alırsanız, hata mesajındaki modülü kurmanız gerekir.

### Build
Kök dizinde:

```bash
cd /path/to/TEKNOFEST2026_fighter_uav_integration

# (opsiyonel) ROS ortamı
source /opt/ros/humble/setup.bash

# bağımlılıklar varsa rosdep ile çekilebilir (opsiyonel)
# rosdep install --from-paths . --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

---

### A) PX4 SITL + Gazebo + Tracking

1) Simülasyonu başlat (Terminal 1)

```bash
# Script içindeki PX4_DIR varsayılanı: ~/Desktop/PX4-Autopilot
# Farklıysa: PX4_DIR=/path/to/PX4-Autopilot ./start_multi_aircraft.sh

cd teknofest_simulation/launch
./start_multi_aircraft.sh
```

2) QGroundControl ile araçları arm + takeoff (HERO + ENEMY’ler)
- Script sonunda gösterilen UDP portları: `14540`, `14541`, `14542`.

3) Tracking node’u başlat (Terminal 2)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Parametre dosyasıyla (önerilen)
ros2 run teknofest_control gps_tracking_node --ros-args \
  --params-file teknofest_control/config/tracking_params.yaml

# Çoklu hedef PX4 namespace’leriyle
# (rakipler: /px4_1, /px4_2)
ros2 run teknofest_control gps_tracking_node --ros-args \
  --params-file teknofest_control/config/tracking_params.yaml \
  -p namespaces.targets:="/px4_1,/px4_2"
```

4) (Opsiyonel) Kamera görüntüsü (Terminal 3)

```bash
cd teknofest_vision/launch
./start_camera_bridge.sh

# görüntülemek için
ros2 run rqt_image_view rqt_image_view /hero/camera/image_raw
```

---

### B) Mock Sunucu + Tracking (PX4 olmadan)
Bu mod; hedef listesi akışını ve takip mantığını **Mock telemetri** ile test etmek içindir.

1) Mock sunucuyu başlat (Terminal 1)

```bash
cd mock/server
python3 ws_server.py
```

2) Mock telemetri client ile drone’ları başlat (Terminal 2)

```bash
python3 mock/clients/ws_test_client.py
```

3) ROS tarafı: mock_target_receiver + gps_tracking_node (Terminal 3)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Mock telemetriyi ROS topic’lerine çevir
ros2 run teknofest_control mock_target_receiver --ros-args \
  -p mock_server_url:=http://localhost:8080 \
  -p own_uav_id:=bizim_iha

# Ayrı terminalde takip
ros2 run teknofest_control gps_tracking_node --ros-args \
  --params-file teknofest_control/config/tracking_params.yaml
```

> Bu modda PX4 topic’leri yoksa `gps_tracking_node` bazı PX4 publisher’larını kullanamayabilir; loglar buna göre değişir.

---

### C) PX4 → Mock → Tracking (karma test)
Amaç: PX4 SITL rakip instance’ların telemetrisini mock sunucuya basıp, oradan tekrar ROS tarafına hedef listesi üretmek.

1) Mock sunucu (Terminal 1)

```bash
cd mock/server
python3 ws_server.py
```

2) PX4 → Mock bridge (Terminal 2)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run teknofest_control px4_to_mock_bridge --ros-args \
  -p mock_server_url:=http://localhost:8080 \
  -p enemy_instance_ids:="[1,2]"
```

3) Mock → ROS hedef listesi (Terminal 3)

```bash
ros2 run teknofest_control mock_target_receiver --ros-args \
  -p mock_server_url:=http://localhost:8080 \
  -p own_uav_id:=bizim_iha
```

4) Tracking (Terminal 4)

```bash
ros2 run teknofest_control gps_tracking_node --ros-args \
  --params-file teknofest_control/config/tracking_params.yaml
```

---

## Mimari / Veri Akışı

### PX4 SITL ile (özet)
- PX4 → (ROS2 topic’leri) → `gps_tracking_node` → (Offboard setpoint) → PX4

PX4’den dinlenen başlıca topic’ler (own namespace’e göre değişebilir):
- `/<ns>/fmu/out/vehicle_local_position` (ve `_v1`)
- `/<ns>/fmu/out/vehicle_global_position`
- `/<ns>/fmu/out/vehicle_attitude`
- `/<ns>/fmu/out/vehicle_status` (ve `_v1`)

PX4’e basılan başlıca topic’ler:
- `/<ns>/fmu/in/trajectory_setpoint`
- `/<ns>/fmu/in/offboard_control_mode`
- `/<ns>/fmu/in/vehicle_command`

### Mock ile (özet)
- `ws_server.py` → (Socket.IO `telemetry`) → `mock_target_receiver`
- `mock_target_receiver` → `/competition/target_list` → `gps_tracking_node`

---

## ROS 2 Topic / Arayüz Referansı

### `gps_tracking_node` – Subscribe
- `/competition/target_list` (`std_msgs/Float64MultiArray`)
  - Format: her hedef için `[id, lat, lon, alt, speed, heading]`
- `/simulation/enemy_aircraft` (`std_msgs/String`)
  - JSON formatı: `{ "aircraft": [ {"id": 1, "lat":..., "lon":..., "alt":..., "speed":..., "heading":...}, ...] }`
- PX4 topic’leri: `VehicleLocalPosition`, `VehicleGlobalPosition`, `VehicleAttitude`, `VehicleStatus` (varsa)

### `gps_tracking_node` – Publish
- PX4 Offboard:
  - `/<ns>/fmu/in/trajectory_setpoint` (`px4_msgs/TrajectorySetpoint`)
  - `/<ns>/fmu/in/offboard_control_mode` (`px4_msgs/OffboardControlMode`)
  - `/<ns>/fmu/in/vehicle_command` (`px4_msgs/VehicleCommand`)
- Debug / gözlem:
  - `/tracking/state` (`std_msgs/String`)
  - `/tracking/target_scores` (`std_msgs/String`) – skor/analiz JSON’u
  - `/tracking/best_target` (`std_msgs/String`) – seçilen hedef
  - `/tracking/camera_fov_status` (`std_msgs/String`)
  - `/tracking/debug_info` (`std_msgs/String`)
  - `/tracking/virtual_target` (`geometry_msgs/PoseStamped`)

### `mock_target_receiver` – Publish
- `/competition/target_list` (`std_msgs/Float64MultiArray`)
- `/simulation/enemy_aircraft` (`std_msgs/String`)
- `/mock/target_count` (`std_msgs/Int32`)

---

## Konfigürasyon

Ana parametre dosyası: `teknofest_control/config/tracking_params.yaml`

Öne çıkan parametre grupları:
- `weights.*`: hedef seçimi ağırlıkları (mesafe/açı/hız/kamera_fov/ulaşılabilirlik)
- `camera_fov.*`: kamera yatay/dikey FOV ve mesafe penceresi
- `target_selection.*`: hysteresis (mevcut hedefe bonus, switch eşiği)
- `l1_guidance.*`: L1 mesafesi, damping, adaptif L1
- `aircraft_limits.*`: min/max/cruise airspeed, bank angle vb.
- `state_machine.*`: yaklaşma/kilit/loiter eşikleri
- `px4.*`: kontrol frekansı, offboard timeout, setpoint modu
- `namespaces.own` ve `namespaces.targets`: multi-vehicle SITL namespace’leri

Namespace örnekleri:
- `namespaces.own: ''` → topic’ler `/fmu/...` altında
- `namespaces.own: '/px4_1'` → topic’ler `/px4_1/fmu/...` altında
- `namespaces.targets: '/px4_1,/px4_2'` → hedef subscriber’ları bu namespace’lerden dinler

---

## Mock Sunucu ve Dashboard

### Mock Sunucu
- Sunucu: `mock/server/ws_server.py`
- Port: **8080** (kod içinde)

HSS bölgeleri:
- `mock/config/hss_zones.json`

### Dashboard
- `mock/dashboard/index.html` + `app.js` ile Leaflet haritası.

> Bilinen nokta: Dashboard ve test client tarafı `http://localhost:8000`’a bağlanıyor; mock sunucu ise 8080’de çalışıyor. Kullanım için `mock/dashboard/app.js` ve `mock/clients/ws_test_client.py` içindeki portu 8080 ile uyumlu hale getirin veya sunucuyu 8000’de çalıştırın.

---

## Sorun Giderme

### 1) PX4 topic’leri gelmiyor
- PX4 SITL gerçekten çalışıyor mu (`pgrep -x px4`)?
- Topic listesi:
  - `ros2 topic list | grep fmu`
- Multi-vehicle ise namespace’ler doğru mu?
  - Örn: `/px4_1/fmu/out/vehicle_global_position`

### 2) Offboard’a geçmiyor / setpoint reject
- PX4 Offboard için belirli sayıda setpoint “ısınması” gerekir.
- QGroundControl’da mod değişimi/arming adımlarını kontrol edin.

### 3) Mock’a bağlanamıyor
- Sunucu çalışıyor mu: `python3 mock/server/ws_server.py`
- URL doğru mu: `http://localhost:8080`
- Bağımlılıklar kurulu mu (`flask_socketio`, `gevent`, `python-socketio`)?

### 4) `start_multi_aircraft.sh` dosya yolu hatası
- Script içinde `PX4_DIR` ve `TEKNOFEST_WS` değişkenleri environment’ınıza göre ayarlanmış olmalı.
- Varsayılanlar:
  - `PX4_DIR=~/Desktop/PX4-Autopilot`
  - `TEKNOFEST_WS=~/teknofest_ws/src/teknofest_simulation`

---

## Bilinen noktalar
- `mock/clients/ws_test_client.py` ve `mock/dashboard/app.js` varsayılan portu `8000` kullanıyor; mock sunucu `8080`.
- `teknofest_control/config/mock_tracking_params.yaml` içinde `nearest_target_tracker` parametre bloğu var; bu repo’da console script olarak tanımlı bir node görünmüyor (legacy/deneysel olabilir).
- `teknofest_control/scripts/start_mock_tracking.sh` bazı launch argümanlarını (örn. `tracking_speed`) geçiriyor; mevcut `mock_tracking.launch.py` bunları DeclareLaunchArgument olarak tanımlamıyor.

---

## Lisans
Apache-2.0 (paket metadata: `teknofest_control/package.xml`).
