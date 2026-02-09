#!/bin/bash
# ================================================================================
# TEKNOFEST SAVAŞAN İHA - TAKİP SİSTEMİ BAŞLATMA SCRIPT'İ
# ================================================================================
# Bu script tüm gerekli node'ları doğru sırayla başlatır.
#
# TOPIC EŞLEŞTİRMELERİ:
# =====================
# PX4 -> gps_tracking_node:
#   /fmu/out/vehicle_local_position   -> own_state (x,y,z,vx,vy,vz,heading)
#   /fmu/out/vehicle_global_position  -> own_state (lat,lon,alt)
#   /fmu/out/vehicle_attitude         -> own_state (roll,pitch)
#   /fmu/out/vehicle_status           -> armed, offboard_mode_active
#
# gps_tracking_node -> PX4:
#   /fmu/in/trajectory_setpoint       <- velocity komutları (NED)
#   /fmu/in/offboard_control_mode     <- offboard heartbeat
#   /fmu/in/vehicle_command           <- ARM/DISARM, mode değişikliği
#
# Mock Sunucu -> gps_tracking_node:
#   /competition/target_list          <- Float64MultiArray [id,lat,lon,alt,speed,heading]
#   /simulation/enemy_aircraft        <- JSON formatı
#
# Debug çıktıları:
#   /tracking/debug_info              -> durum bilgisi
#   /tracking/state                   -> TrackingState enum
#   /tracking/virtual_target          -> L1 sanal hedef noktası (PoseStamped)
#
# Kullanım:
#   ./baslat.sh                # Tüm sistemi başlat
#   ./baslat.sh --mock-only    # Sadece mock sunucu ile test
#   ./baslat.sh --sim          # Simülasyon modu (PX4 SITL)
# ================================================================================

set -e

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Varsayılan değerler
MODE="full"
MOCK_PORT=8080
PX4_INSTANCE=0

# Argümanları parse et
while [[ $# -gt 0 ]]; do
    case $1 in
        --mock-only)
            MODE="mock"
            shift
            ;;
        --sim)
            MODE="sim"
            shift
            ;;
        --port)
            MOCK_PORT="$2"
            shift 2
            ;;
        --instance)
            PX4_INSTANCE="$2"
            shift 2
            ;;
        *)
            echo -e "${RED}Bilinmeyen argüman: $1${NC}"
            exit 1
            ;;
    esac
done

# Banner
echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║     TEKNOFEST SAVAŞAN İHA - TAKİP SİSTEMİ                  ║"
echo "║                                                            ║"
echo "║  L1 Adaptive Guidance + Fixed-Wing Control                 ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Script dizini
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

# ROS2'yi source et
echo -e "${YELLOW}📦 ROS2 ortamı yükleniyor...${NC}"

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}   ✓ ROS2 Humble${NC}"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo -e "${GREEN}   ✓ ROS2 Foxy${NC}"
else
    echo -e "${RED}   ✗ ROS2 bulunamadı!${NC}"
    exit 1
fi

# Workspace'i source et
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
    echo -e "${GREEN}   ✓ Workspace: ${WS_DIR}${NC}"
else
    echo -e "${YELLOW}   ⚠ Workspace build edilmemiş. Derleniyor...${NC}"
    cd "${WS_DIR}"
    colcon build --packages-select teknofest_control --symlink-install
    source "${WS_DIR}/install/setup.bash"
fi

# PX4 namespace (multi-vehicle için)
if [ $PX4_INSTANCE -gt 0 ]; then
    PX4_NS="/px4_${PX4_INSTANCE}"
    echo -e "${BLUE}   PX4 Instance: ${PX4_INSTANCE} (namespace: ${PX4_NS})${NC}"
else
    PX4_NS=""
fi

echo ""

# ============================================================================
# TOPIC DOĞRULAMA
# ============================================================================
echo -e "${YELLOW}🔍 Topic bağlantıları kontrol ediliyor...${NC}"

check_topic() {
    local topic=$1
    local timeout=2
    if ros2 topic info "$topic" --no-daemon 2>/dev/null | grep -q "Publisher"; then
        echo -e "${GREEN}   ✓ $topic${NC}"
        return 0
    else
        echo -e "${YELLOW}   ⏳ $topic (bekleniyor)${NC}"
        return 1
    fi
}

# PX4 topic'lerini kontrol et (simülasyon modunda)
if [ "$MODE" = "sim" ] || [ "$MODE" = "full" ]; then
    echo -e "${CYAN}   PX4 Topic'leri:${NC}"
    check_topic "${PX4_NS}/fmu/out/vehicle_local_position" || true
    check_topic "${PX4_NS}/fmu/out/vehicle_global_position" || true
    check_topic "${PX4_NS}/fmu/out/vehicle_status" || true
fi

echo ""

# ============================================================================
# NODE'LARI BAŞLAT
# ============================================================================

cleanup() {
    echo -e "\n${YELLOW}🛑 Sistem kapatılıyor...${NC}"
    pkill -f "mock_target_receiver" 2>/dev/null || true
    pkill -f "gps_tracking_node" 2>/dev/null || true
    pkill -f "mock_server_bridge" 2>/dev/null || true
    echo -e "${GREEN}✓ Temizlik tamamlandı${NC}"
}

trap cleanup EXIT INT TERM

case $MODE in
    "mock")
        echo -e "${BLUE}🎯 Mock Modu - Sadece simülasyon verisi ile test${NC}"
        echo ""
        
        # Mock target receiver başlat
        echo -e "${GREEN}▶ Mock Target Receiver başlatılıyor...${NC}"
        ros2 run teknofest_control mock_target_receiver \
            --ros-args \
            -p mock_server_url:="http://localhost:${MOCK_PORT}" \
            -p own_uav_id:="bizim_iha" \
            -p publish_rate_hz:=50.0 &
        
        sleep 2
        
        # GPS Tracking Node başlat
        echo -e "${GREEN}▶ GPS Tracking Node başlatılıyor...${NC}"
        ros2 run teknofest_control gps_tracking_node \
            --ros-args \
            -p l1_guidance.l1_distance:=50.0 \
            -p l1_guidance.adaptive_l1:=true \
            -p aircraft_limits.cruise_airspeed:=22.0 \
            -p px4.control_frequency:=50.0
        ;;
        
    "sim")
        echo -e "${BLUE}🛩️ Simülasyon Modu - PX4 SITL ile test${NC}"
        echo ""
        
        # Topic remapping ile başlat (namespace desteği)
        if [ -n "$PX4_NS" ]; then
            REMAP_ARGS="--remap /fmu/out/vehicle_local_position:=${PX4_NS}/fmu/out/vehicle_local_position \
                        --remap /fmu/out/vehicle_global_position:=${PX4_NS}/fmu/out/vehicle_global_position \
                        --remap /fmu/out/vehicle_attitude:=${PX4_NS}/fmu/out/vehicle_attitude \
                        --remap /fmu/out/vehicle_status:=${PX4_NS}/fmu/out/vehicle_status \
                        --remap /fmu/in/trajectory_setpoint:=${PX4_NS}/fmu/in/trajectory_setpoint \
                        --remap /fmu/in/offboard_control_mode:=${PX4_NS}/fmu/in/offboard_control_mode \
                        --remap /fmu/in/vehicle_command:=${PX4_NS}/fmu/in/vehicle_command"
        else
            REMAP_ARGS=""
        fi
        
        echo -e "${GREEN}▶ GPS Tracking Node başlatılıyor...${NC}"
        ros2 run teknofest_control gps_tracking_node \
            --ros-args \
            $REMAP_ARGS \
            -p l1_guidance.l1_distance:=50.0 \
            -p l1_guidance.adaptive_l1:=true \
            -p aircraft_limits.min_airspeed:=15.0 \
            -p aircraft_limits.max_airspeed:=35.0 \
            -p aircraft_limits.cruise_airspeed:=22.0 \
            -p px4.control_frequency:=50.0
        ;;
        
    "full")
        echo -e "${BLUE}🚀 Tam Mod - Mock + PX4${NC}"
        echo ""
        
        # Mock target receiver
        echo -e "${GREEN}▶ Mock Target Receiver başlatılıyor...${NC}"
        ros2 run teknofest_control mock_target_receiver \
            --ros-args \
            -p mock_server_url:="http://localhost:${MOCK_PORT}" \
            -p own_uav_id:="bizim_iha" \
            -p publish_rate_hz:=50.0 &
        
        sleep 2
        
        # GPS Tracking Node (varsayılan namespace)
        echo -e "${GREEN}▶ GPS Tracking Node başlatılıyor...${NC}"
        ros2 run teknofest_control gps_tracking_node \
            --ros-args \
            -p l1_guidance.l1_distance:=50.0 \
            -p l1_guidance.adaptive_l1:=true \
            -p aircraft_limits.min_airspeed:=15.0 \
            -p aircraft_limits.max_airspeed:=35.0 \
            -p aircraft_limits.cruise_airspeed:=22.0 \
            -p px4.control_frequency:=50.0
        ;;
esac

echo -e "${GREEN}✓ Sistem başlatıldı${NC}"
