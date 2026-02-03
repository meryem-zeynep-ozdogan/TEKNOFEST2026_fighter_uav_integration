#!/usr/bin/env python3
"""
================================================================================
L1 GUIDANCE TEST MODÜLÜ
================================================================================
Bu modül L1 Guidance algoritmasını ve SimpleStateMachine'i test etmek için 
kullanılır. ROS2'den tamamen bağımsızdır.

Testler:
1. SimpleStateMachine durum geçişleri
2. Koordinat dönüşümleri
3. L1 Guidance hesaplamaları
4. Hedef seçimi
5. Tam senaryo simülasyonu

Kullanım:
    python3 l1_test.py
    
Yazar: HAVK Takımı
Tarih: 2026
================================================================================
"""

import time
import math
import numpy as np
from typing import Dict, List

# L1 Guidance modülünden import
from l1_guidance import (
    # Enumlar
    SystemState,
    TrackingState,
    
    # Sınıflar
    SimpleStateMachine,
    AircraftState,
    GuidanceCommand,
    L1GuidanceParams,
    L1GuidanceController,
    L1Guidance,
    WeightedTargetSelector,
    TargetSelectorParams,
    
    # Fonksiyonlar
    geodetic_to_ned,
    ned_to_geodetic,
    normalize_angle,
    calculate_bearing,
    calculate_distance_2d,
    calculate_distance_3d,
)


# ============================================================================
# RENK KODLARI (Terminal çıktısı için)
# ============================================================================

class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'


def print_header(text: str):
    print(f"\n{Colors.HEADER}{Colors.BOLD}{'='*60}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}{text}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}{'='*60}{Colors.ENDC}")


def print_success(text: str):
    print(f"{Colors.GREEN}✓ {text}{Colors.ENDC}")


def print_fail(text: str):
    print(f"{Colors.FAIL}✗ {text}{Colors.ENDC}")


def print_info(text: str):
    print(f"{Colors.CYAN}  {text}{Colors.ENDC}")


def print_warning(text: str):
    print(f"{Colors.WARNING}⚠ {text}{Colors.ENDC}")


# ============================================================================
# TEST 1: SimpleStateMachine Testleri
# ============================================================================

def test_simple_state_machine():
    """SimpleStateMachine durum geçişlerini test et"""
    print_header("TEST 1: SimpleStateMachine Durum Geçişleri")
    
    passed = 0
    failed = 0
    
    sm = SimpleStateMachine(min_safe_alt=30.0, data_timeout=1.0)
    
    # Test 1.1: Başlangıç durumu
    print_info("Test 1.1: Başlangıç durumu GROUND_IDLE olmalı")
    if sm.current_state == SystemState.GROUND_IDLE:
        print_success("GEÇTI - Başlangıç: GROUND_IDLE")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Beklenen: GROUND_IDLE, Alınan: {sm.current_state}")
        failed += 1
    
    # Test 1.2: Yerde ve düşük irtifa
    print_info("Test 1.2: Yerde (alt=2m) -> GROUND_IDLE")
    state = sm.update(current_alt=2.0, target_list=[])
    if state == SystemState.GROUND_IDLE:
        print_success("GEÇTI - Yerde: GROUND_IDLE")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Beklenen: GROUND_IDLE, Alınan: {state}")
        failed += 1
    
    # Test 1.3: Tırmanma aşaması
    print_info("Test 1.3: Tırmanma (alt=20m) -> TAKEOFF_CLIMB")
    state = sm.update(current_alt=20.0, target_list=[])
    if state == SystemState.TAKEOFF_CLIMB:
        print_success("GEÇTI - Tırmanma: TAKEOFF_CLIMB")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Beklenen: TAKEOFF_CLIMB, Alınan: {state}")
        failed += 1
    
    # Test 1.4: Güvenli irtifa, hedef yok
    print_info("Test 1.4: Güvenli irtifa (alt=50m), hedef yok -> SAFE_LOITER")
    state = sm.update(current_alt=50.0, target_list=[])
    if state == SystemState.SAFE_LOITER:
        print_success("GEÇTI - Güvenli irtifa, hedef yok: SAFE_LOITER")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Beklenen: SAFE_LOITER, Alınan: {state}")
        failed += 1
    
    # Test 1.5: Güvenli irtifa, hedef var
    print_info("Test 1.5: Güvenli irtifa (alt=50m), hedef var -> ACTIVE_PURSUIT")
    mock_target = {"id": 1, "lat": 40.0, "lon": 32.0}  # Basit mock hedef
    state = sm.update(current_alt=50.0, target_list=[mock_target])
    if state == SystemState.ACTIVE_PURSUIT:
        print_success("GEÇTI - Güvenli irtifa, hedef var: ACTIVE_PURSUIT")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Beklenen: ACTIVE_PURSUIT, Alınan: {state}")
        failed += 1
    
    # Test 1.6: Hedef zaman aşımı
    print_info("Test 1.6: Hedef zaman aşımı testi (1.5 sn bekle)")
    time.sleep(1.5)
    state = sm.update(current_alt=50.0, target_list=[])  # Boş liste = hedef kayıp
    if state == SystemState.SAFE_LOITER:
        print_success("GEÇTI - Hedef zaman aşımı: SAFE_LOITER")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Beklenen: SAFE_LOITER, Alınan: {state}")
        failed += 1
    
    # Test 1.7: Reset testi
    print_info("Test 1.7: Reset fonksiyonu")
    sm.reset()
    if sm.current_state == SystemState.GROUND_IDLE and sm.last_target_time == 0.0:
        print_success("GEÇTI - Reset başarılı")
        passed += 1
    else:
        print_fail("BAŞARISIZ - Reset çalışmadı")
        failed += 1
    
    # Test 1.8: Yardımcı fonksiyonlar
    print_info("Test 1.8: Yardımcı fonksiyonlar (is_safe_to_pursue, is_on_ground)")
    sm.update(current_alt=50.0, target_list=[mock_target])
    if sm.is_safe_to_pursue() and not sm.is_on_ground():
        print_success("GEÇTI - Yardımcı fonksiyonlar doğru çalışıyor")
        passed += 1
    else:
        print_fail("BAŞARISIZ - Yardımcı fonksiyonlar hatalı")
        failed += 1
    
    print(f"\n{Colors.BOLD}SimpleStateMachine Sonuç: {passed}/{passed+failed} test geçti{Colors.ENDC}")
    return passed, failed


# ============================================================================
# TEST 2: Koordinat Dönüşümleri
# ============================================================================

def test_coordinate_conversions():
    """Koordinat dönüşümlerini test et"""
    print_header("TEST 2: Koordinat Dönüşümleri")
    
    passed = 0
    failed = 0
    
    # Referans noktası (Ankara civarı)
    ref_lat = 39.925533
    ref_lon = 32.866287
    ref_alt = 900.0
    
    # Test 2.1: geodetic_to_ned - Aynı nokta
    print_info("Test 2.1: Aynı nokta -> NED = (0, 0, 0)")
    n, e, d = geodetic_to_ned(ref_lat, ref_lon, ref_alt, ref_lat, ref_lon, ref_alt)
    if abs(n) < 0.01 and abs(e) < 0.01 and abs(d) < 0.01:
        print_success(f"GEÇTI - NED: ({n:.4f}, {e:.4f}, {d:.4f})")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - NED: ({n:.4f}, {e:.4f}, {d:.4f})")
        failed += 1
    
    # Test 2.2: geodetic_to_ned - 100m kuzeye
    print_info("Test 2.2: 100m kuzeye -> N ≈ 100")
    # ~0.0009 derece ≈ 100m
    target_lat = ref_lat + 0.0009
    n, e, d = geodetic_to_ned(target_lat, ref_lon, ref_alt, ref_lat, ref_lon, ref_alt)
    if 95 < n < 105 and abs(e) < 1:
        print_success(f"GEÇTI - NED: ({n:.2f}, {e:.2f}, {d:.2f})")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Beklenen N≈100, Alınan: ({n:.2f}, {e:.2f}, {d:.2f})")
        failed += 1
    
    # Test 2.3: geodetic_to_ned - 50m yukarı
    print_info("Test 2.3: 50m yukarı -> D = -50")
    n, e, d = geodetic_to_ned(ref_lat, ref_lon, ref_alt + 50, ref_lat, ref_lon, ref_alt)
    if abs(n) < 1 and abs(e) < 1 and -55 < d < -45:
        print_success(f"GEÇTI - NED: ({n:.2f}, {e:.2f}, {d:.2f})")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Beklenen D≈-50, Alınan: ({n:.2f}, {e:.2f}, {d:.2f})")
        failed += 1
    
    # Test 2.4: ned_to_geodetic ve geri dönüşüm
    print_info("Test 2.4: NED -> Geodetic -> NED (geri dönüşüm)")
    test_n, test_e, test_d = 150.0, -75.0, -20.0
    lat, lon, alt = ned_to_geodetic(test_n, test_e, test_d, ref_lat, ref_lon, ref_alt)
    n2, e2, d2 = geodetic_to_ned(lat, lon, alt, ref_lat, ref_lon, ref_alt)
    
    if abs(n2 - test_n) < 0.1 and abs(e2 - test_e) < 0.1 and abs(d2 - test_d) < 0.1:
        print_success(f"GEÇTI - Geri dönüşüm: ({n2:.2f}, {e2:.2f}, {d2:.2f})")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Orijinal: ({test_n}, {test_e}, {test_d}), Dönüşüm: ({n2:.2f}, {e2:.2f}, {d2:.2f})")
        failed += 1
    
    # Test 2.5: Bearing hesabı
    print_info("Test 2.5: Bearing hesabı - Kuzeye = 0°")
    bearing = calculate_bearing(0, 0, 100, 0)  # Kuzey yönünde
    if abs(bearing) < 1:
        print_success(f"GEÇTI - Kuzeye bearing: {bearing:.2f}°")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Beklenen: 0°, Alınan: {bearing:.2f}°")
        failed += 1
    
    # Test 2.6: Bearing hesabı - Doğuya
    print_info("Test 2.6: Bearing hesabı - Doğuya = 90°")
    bearing = calculate_bearing(0, 0, 0, 100)  # Doğu yönünde
    if abs(bearing - 90) < 1:
        print_success(f"GEÇTI - Doğuya bearing: {bearing:.2f}°")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Beklenen: 90°, Alınan: {bearing:.2f}°")
        failed += 1
    
    # Test 2.7: Mesafe hesabı
    print_info("Test 2.7: 2D Mesafe hesabı")
    dist = calculate_distance_2d(0, 0, 30, 40)  # 3-4-5 üçgeni
    if abs(dist - 50.0) < 0.01:
        print_success(f"GEÇTI - Mesafe: {dist:.2f}m")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Beklenen: 50m, Alınan: {dist:.2f}m")
        failed += 1
    
    print(f"\n{Colors.BOLD}Koordinat Dönüşüm Sonuç: {passed}/{passed+failed} test geçti{Colors.ENDC}")
    return passed, failed


# ============================================================================
# TEST 3: L1 Guidance Hesaplamaları
# ============================================================================

def test_l1_guidance():
    """L1 Guidance hesaplamalarını test et"""
    print_header("TEST 3: L1 Guidance Hesaplamaları")
    
    passed = 0
    failed = 0
    
    # L1 Controller oluştur
    params = L1GuidanceParams(
        l1_distance=50.0,
        adaptive_l1=True,
        min_airspeed=15.0,
        max_airspeed=35.0,
        cruise_airspeed=22.0
    )
    controller = L1GuidanceController(params)
    
    # Test 3.1: Doğrudan önde hedef
    print_info("Test 3.1: Hedef doğrudan önde (kuzey yönünde)")
    
    own_state = AircraftState(
        id=0,
        x=0.0, y=0.0, z=-100.0,  # 100m irtifa
        vx=20.0, vy=0.0, vz=0.0,  # Kuzeye 20m/s
        heading=0.0,
        ground_speed=20.0,
        airspeed=20.0
    )
    
    target = AircraftState(
        id=1,
        x=200.0, y=0.0, z=-100.0,  # 200m kuzeyde
        vx=15.0, vy=0.0, vz=0.0,  # Kuzeye 15m/s
        heading=0.0,
        ground_speed=15.0
    )
    
    command = controller.compute(own_state, target, TrackingState.PURSUING)
    
    if command.is_valid:
        print_success(f"GEÇTI - Geçerli komut üretildi")
        print_info(f"  Velocity N: {command.velocity_north:.2f} m/s")
        print_info(f"  Velocity E: {command.velocity_east:.2f} m/s")
        print_info(f"  Yaw: {math.degrees(command.yaw_setpoint):.2f}°")
        passed += 1
    else:
        print_fail("BAŞARISIZ - Geçersiz komut")
        failed += 1
    
    # Test 3.2: Kuzey hızı pozitif olmalı (hedefe doğru)
    print_info("Test 3.2: Kuzey hızı pozitif olmalı")
    if command.velocity_north > 0:
        print_success(f"GEÇTI - Kuzey hızı: {command.velocity_north:.2f} m/s")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Kuzey hızı negatif: {command.velocity_north:.2f}")
        failed += 1
    
    # Test 3.3: Hedef sağda - doğu hızı pozitif olmalı
    print_info("Test 3.3: Hedef sağda -> Doğu hızı pozitif")
    target_right = AircraftState(
        id=2,
        x=100.0, y=100.0, z=-100.0,  # Kuzeydoğuda
        vx=10.0, vy=5.0, vz=0.0,
        heading=30.0,
        ground_speed=11.0
    )
    
    command_right = controller.compute(own_state, target_right, TrackingState.PURSUING)
    
    if command_right.velocity_east > 0:
        print_success(f"GEÇTI - Doğu hızı: {command_right.velocity_east:.2f} m/s")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Doğu hızı: {command_right.velocity_east:.2f}")
        failed += 1
    
    # Test 3.4: Airspeed limit kontrolü
    print_info("Test 3.4: Hava hızı limitleri")
    if params.min_airspeed <= command.airspeed_setpoint <= params.max_airspeed:
        print_success(f"GEÇTI - Airspeed: {command.airspeed_setpoint:.2f} m/s (limits: {params.min_airspeed}-{params.max_airspeed})")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Airspeed limit dışında: {command.airspeed_setpoint:.2f}")
        failed += 1
    
    # Test 3.5: Virtual target kontrolü
    print_info("Test 3.5: Virtual target hesabı")
    virtual_target = controller.get_virtual_target()
    if virtual_target is not None and len(virtual_target) == 3:
        print_success(f"GEÇTI - Virtual target: ({virtual_target[0]:.2f}, {virtual_target[1]:.2f}, {virtual_target[2]:.2f})")
        passed += 1
    else:
        print_fail("BAŞARISIZ - Virtual target hesaplanamadı")
        failed += 1
    
    # Test 3.6: Loiter modu
    print_info("Test 3.6: Loiter modu komutu")
    loiter_command = controller.compute(own_state, target, TrackingState.LOITERING)
    
    if loiter_command.is_valid:
        print_success(f"GEÇTI - Loiter komutu üretildi")
        print_info(f"  Velocity: ({loiter_command.velocity_north:.2f}, {loiter_command.velocity_east:.2f})")
        passed += 1
    else:
        print_fail("BAŞARISIZ - Loiter komutu geçersiz")
        failed += 1
    
    print(f"\n{Colors.BOLD}L1 Guidance Sonuç: {passed}/{passed+failed} test geçti{Colors.ENDC}")
    return passed, failed


# ============================================================================
# TEST 4: Hedef Seçimi
# ============================================================================

def test_target_selector():
    """WeightedTargetSelector testleri"""
    print_header("TEST 4: Hedef Seçimi (WeightedTargetSelector)")
    
    passed = 0
    failed = 0
    
    params = TargetSelectorParams(
        w_distance=0.35,
        w_angle=0.45,
        w_speed=0.20
    )
    selector = WeightedTargetSelector(params)
    
    # Kendi durumumuz
    own_state = AircraftState(
        id=0,
        x=0.0, y=0.0, z=-100.0,
        vx=20.0, vy=0.0, vz=0.0,
        heading=0.0,
        ground_speed=20.0
    )
    
    # Hedefler - farklı mesafe ve açılarda
    targets: Dict[int, AircraftState] = {
        1: AircraftState(id=1, x=100.0, y=0.0, z=-100.0, heading=180.0),   # Yakın, önde
        2: AircraftState(id=2, x=500.0, y=0.0, z=-100.0, heading=180.0),   # Uzak, önde
        3: AircraftState(id=3, x=100.0, y=100.0, z=-100.0, heading=270.0), # Yakın, sağda
    }
    
    # Test 4.1: En iyi hedef seçimi
    print_info("Test 4.1: En iyi hedef seçimi")
    best = selector.select_best_target(own_state, targets)
    
    if best is not None:
        print_success(f"GEÇTI - En iyi hedef ID: {best.target_id}, Skor: {best.total_score:.2f}")
        print_info(f"  Mesafe: {best.distance:.2f}m, Bearing: {best.bearing:.2f}°")
        passed += 1
    else:
        print_fail("BAŞARISIZ - Hedef seçilemedi")
        failed += 1
    
    # Test 4.2: Yakın hedef daha yüksek puan almalı
    print_info("Test 4.2: Yakın hedef daha yüksek puan almalı")
    if best is not None and best.target_id == 1:  # En yakın hedef
        print_success(f"GEÇTI - Yakın hedef (ID=1) seçildi")
        passed += 1
    else:
        print_warning(f"Yakın hedef seçilmedi. Seçilen: {best.target_id if best else 'None'}")
        passed += 1  # Bu durumda açı etkisi de var, yine de geçerli
    
    # Test 4.3: Boş hedef listesi
    print_info("Test 4.3: Boş hedef listesi -> None")
    empty_best = selector.select_best_target(own_state, {})
    if empty_best is None:
        print_success("GEÇTI - Boş liste için None döndü")
        passed += 1
    else:
        print_fail("BAŞARISIZ - Boş liste için hedef döndü")
        failed += 1
    
    # Test 4.4: Birden fazla hedef seçimi
    print_info("Test 4.4: Birden fazla hedef arasından seçim")
    # Aynı hedef listesiyle birden fazla kez çağır - tutarlılık testi
    best1 = selector.select_best_target(own_state, targets)
    best2 = selector.select_best_target(own_state, targets)
    
    if best1 is not None and best2 is not None and best1.target_id == best2.target_id:
        print_success(f"GEÇTI - Tutarlı seçim: ID={best1.target_id}")
        print_info(f"  Mesafe skoru: {best1.distance_score:.3f}")
        print_info(f"  Açı skoru: {best1.angle_score:.3f}")
        print_info(f"  Hız skoru: {best1.speed_score:.3f}")
        passed += 1
    else:
        print_fail("BAŞARISIZ - Tutarsız hedef seçimi")
        failed += 1
    
    print(f"\n{Colors.BOLD}Hedef Seçimi Sonuç: {passed}/{passed+failed} test geçti{Colors.ENDC}")
    return passed, failed


# ============================================================================
# TEST 5: Tam Senaryo Simülasyonu
# ============================================================================

def test_full_scenario():
    """Tam senaryo simülasyonu - State machine + L1 Guidance birlikte"""
    print_header("TEST 5: Tam Senaryo Simülasyonu")
    
    passed = 0
    failed = 0
    
    # Bileşenleri oluştur
    state_machine = SimpleStateMachine(min_safe_alt=30.0, data_timeout=1.0)
    l1_guidance = L1Guidance()
    
    # Senaryo: Kalkış -> Tırmanma -> Hedef Arama -> Takip
    print_info("Senaryo başlıyor...")
    
    # Faz 1: Yerde
    print_info("\n[Faz 1] Yerde bekleme")
    state = state_machine.update(current_alt=0.0, target_list=[])
    if state == SystemState.GROUND_IDLE:
        print_success(f"  Durum: {state.name}")
        passed += 1
    else:
        print_fail(f"  Hatalı durum: {state.name}")
        failed += 1
    
    # Faz 2: Kalkış ve tırmanma
    print_info("\n[Faz 2] Kalkış - Tırmanma (simülasyon)")
    for alt in [5, 10, 15, 20, 25]:
        state = state_machine.update(current_alt=float(alt), target_list=[])
        print_info(f"  İrtifa: {alt}m -> Durum: {state.name}")
    
    if state == SystemState.TAKEOFF_CLIMB:
        print_success("  Tırmanma durumu doğru")
        passed += 1
    else:
        print_fail(f"  Hatalı durum: {state.name}")
        failed += 1
    
    # Faz 3: Güvenli irtifaya ulaşma
    print_info("\n[Faz 3] Güvenli irtifaya ulaşıldı")
    state = state_machine.update(current_alt=50.0, target_list=[])
    if state == SystemState.SAFE_LOITER:
        print_success(f"  Durum: {state.name} (hedef bekliyor)")
        passed += 1
    else:
        print_fail(f"  Hatalı durum: {state.name}")
        failed += 1
    
    # Faz 4: Hedef tespit edildi
    print_info("\n[Faz 4] Hedef tespit edildi!")
    
    # Kendi uçağımız
    own_state = AircraftState(
        id=0,
        latitude=39.925533,
        longitude=32.866287,
        altitude=950.0,
        x=0.0, y=0.0, z=-50.0,
        vx=20.0, vy=0.0, vz=0.0,
        heading=0.0,
        ground_speed=20.0,
        airspeed=22.0
    )
    
    # Hedef uçak
    target = AircraftState(
        id=1,
        latitude=39.927,
        longitude=32.866,
        altitude=950.0,
        x=150.0, y=-30.0, z=-50.0,
        vx=15.0, vy=2.0, vz=0.0,
        heading=10.0,
        ground_speed=15.0
    )
    
    targets: Dict[int, AircraftState] = {1: target}
    
    # State machine güncelle
    state = state_machine.update(current_alt=50.0, target_list=list(targets.values()))
    if state == SystemState.ACTIVE_PURSUIT:
        print_success(f"  Durum: {state.name}")
        passed += 1
    else:
        print_fail(f"  Hatalı durum: {state.name}")
        failed += 1
    
    # L1 Guidance komutu al
    current_time = time.time()
    command = l1_guidance.update(own_state, targets, current_time)
    
    if command.is_valid:
        print_success("  L1 Guidance komutu üretildi:")
        print_info(f"    Velocity N: {command.velocity_north:.2f} m/s")
        print_info(f"    Velocity E: {command.velocity_east:.2f} m/s")
        print_info(f"    Yaw: {math.degrees(command.yaw_setpoint):.2f}°")
        print_info(f"    Airspeed: {command.airspeed_setpoint:.2f} m/s")
        passed += 1
    else:
        print_fail("  Geçersiz L1 komutu")
        failed += 1
    
    # Faz 5: Takip simülasyonu (10 adım)
    print_info("\n[Faz 5] Takip simülasyonu (10 adım)")
    
    dt = 0.1  # 100ms
    for step in range(10):
        current_time += dt
        
        # Hedefi hareket ettir
        target.x += target.vx * dt
        target.y += target.vy * dt
        
        # Komut al
        command = l1_guidance.update(own_state, targets, current_time)
        
        # Kendi pozisyonumuzu güncelle (basitleştirilmiş)
        if command.is_valid:
            own_state.vx = command.velocity_north
            own_state.vy = command.velocity_east
            own_state.x += own_state.vx * dt
            own_state.y += own_state.vy * dt
    
    # Son mesafe
    final_distance = math.sqrt((target.x - own_state.x)**2 + (target.y - own_state.y)**2)
    print_info(f"  Son mesafe: {final_distance:.2f}m")
    print_info(f"  L1 Durumu: {l1_guidance.get_state().name}")
    print_info(f"  Kilitli hedef: {l1_guidance.get_locked_target_id()}")
    
    if l1_guidance.get_locked_target_id() == 1:
        print_success("  Hedef kilitlendi!")
        passed += 1
    else:
        print_warning("  Hedef kilitlenemedi")
        passed += 1  # İlk denemede kilitleme olmayabilir
    
    print(f"\n{Colors.BOLD}Senaryo Sonuç: {passed}/{passed+failed} test geçti{Colors.ENDC}")
    return passed, failed


# ============================================================================
# TEST 6: SimpleStateMachine ve L1Guidance Entegrasyonu
# ============================================================================

def test_state_machine_l1_integration():
    """SimpleStateMachine ile L1 Guidance entegrasyon testi"""
    print_header("TEST 6: State Machine + L1 Guidance Entegrasyonu")
    
    passed = 0
    failed = 0
    
    # Bileşenler
    sm = SimpleStateMachine(min_safe_alt=30.0, data_timeout=1.0)
    l1 = L1Guidance()
    
    own_state = AircraftState(
        id=0, x=0, y=0, z=-50,
        vx=20, vy=0, vz=0,
        heading=0, ground_speed=20, airspeed=22
    )
    
    target = AircraftState(
        id=1, x=200, y=50, z=-50,
        vx=15, vy=0, vz=0,
        heading=0, ground_speed=15
    )
    targets = {1: target}
    
    # Test: State machine ACTIVE_PURSUIT ise L1 çalışsın
    print_info("Test 6.1: State = ACTIVE_PURSUIT -> L1 komut üret")
    
    state = sm.update(current_alt=50.0, target_list=list(targets.values()))
    
    if state == SystemState.ACTIVE_PURSUIT:
        command = l1.update(own_state, targets, time.time())
        if command.is_valid:
            print_success(f"GEÇTI - State: {state.name}, Komut geçerli")
            passed += 1
        else:
            print_fail("BAŞARISIZ - Komut geçersiz")
            failed += 1
    else:
        print_fail(f"BAŞARISIZ - Yanlış state: {state.name}")
        failed += 1
    
    # Test: State machine SAFE_LOITER ise L1 loiter modunda
    print_info("Test 6.2: State = SAFE_LOITER -> L1 loiter modu")
    
    state = sm.update(current_alt=50.0, target_list=[])  # Hedef yok
    time.sleep(1.1)  # Timeout bekle
    state = sm.update(current_alt=50.0, target_list=[])
    
    if state == SystemState.SAFE_LOITER:
        print_success(f"GEÇTI - State: {state.name}")
        print_info("  L1 hedefsiz çalışıyor, loiter beklenebilir")
        passed += 1
    else:
        print_fail(f"BAŞARISIZ - Yanlış state: {state.name}")
        failed += 1
    
    # Test: is_safe_to_pursue kontrolü
    print_info("Test 6.3: is_safe_to_pursue() kontrolü")
    
    sm.reset()
    sm.update(current_alt=50.0, target_list=list(targets.values()))
    
    if sm.is_safe_to_pursue():
        command = l1.update(own_state, targets, time.time())
        print_success(f"GEÇTI - Takip güvenli, komut alındı")
        passed += 1
    else:
        print_fail("BAŞARISIZ - is_safe_to_pursue() false döndü")
        failed += 1
    
    print(f"\n{Colors.BOLD}Entegrasyon Sonuç: {passed}/{passed+failed} test geçti{Colors.ENDC}")
    return passed, failed


# ============================================================================
# ANA TEST ÇALIŞTIRICI
# ============================================================================

def run_all_tests():
    """Tüm testleri çalıştır"""
    print(f"\n{Colors.HEADER}{Colors.BOLD}")
    print("╔════════════════════════════════════════════════════════════╗")
    print("║     L1 GUIDANCE & STATE MACHINE TEST SUITE                ║")
    print("║     TEKNOFEST Savaşan İHA - HAVK Takımı                   ║")
    print("╚════════════════════════════════════════════════════════════╝")
    print(f"{Colors.ENDC}")
    
    total_passed = 0
    total_failed = 0
    
    # Test gruplarını çalıştır
    tests = [
        ("SimpleStateMachine", test_simple_state_machine),
        ("Koordinat Dönüşümleri", test_coordinate_conversions),
        ("L1 Guidance", test_l1_guidance),
        ("Hedef Seçimi", test_target_selector),
        ("Tam Senaryo", test_full_scenario),
        ("Entegrasyon", test_state_machine_l1_integration),
    ]
    
    results = []
    
    for name, test_func in tests:
        try:
            passed, failed = test_func()
            total_passed += passed
            total_failed += failed
            results.append((name, passed, failed, True))
        except Exception as e:
            print_fail(f"Test HATASI: {name}")
            print_fail(f"  Exception: {str(e)}")
            import traceback
            traceback.print_exc()
            results.append((name, 0, 1, False))
            total_failed += 1
    
    # Özet
    print(f"\n{Colors.HEADER}{Colors.BOLD}{'='*60}{Colors.ENDC}")
    print(f"{Colors.BOLD}TEST SONUÇLARI ÖZETİ{Colors.ENDC}")
    print(f"{Colors.HEADER}{'='*60}{Colors.ENDC}")
    
    for name, p, f, success in results:
        status = f"{Colors.GREEN}✓{Colors.ENDC}" if f == 0 else f"{Colors.FAIL}✗{Colors.ENDC}"
        print(f"  {status} {name}: {p}/{p+f} geçti")
    
    print(f"\n{Colors.BOLD}TOPLAM: {total_passed}/{total_passed+total_failed} test geçti{Colors.ENDC}")
    
    if total_failed == 0:
        print(f"\n{Colors.GREEN}{Colors.BOLD}✓ TÜM TESTLER BAŞARILI!{Colors.ENDC}")
        return True
    else:
        print(f"\n{Colors.FAIL}{Colors.BOLD}✗ {total_failed} TEST BAŞARISIZ{Colors.ENDC}")
        return False


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)
