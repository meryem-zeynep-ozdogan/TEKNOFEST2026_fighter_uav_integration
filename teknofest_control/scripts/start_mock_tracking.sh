#!/bin/bash
# ================================================================================
# MOCK TARGET TRACKING BAŞLATMA SCRIPT'İ
# ================================================================================
# Bu script Mock sunucu ve ROS2 takip sistemini başlatır.
#
# Kullanım:
#   ./start_mock_tracking.sh [mock_port]
#
# Örnek:
#   ./start_mock_tracking.sh 8080
# ================================================================================

set -e

# Varsayılan değerler
MOCK_PORT=${1:-8080}
MOCK_URL="http://localhost:${MOCK_PORT}"

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "=========================================="
echo "  MOCK TARGET TRACKING SYSTEM"
echo "=========================================="
echo -e "${NC}"

# ROS2 workspace'i source et
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

echo -e "${YELLOW}📦 ROS2 workspace source ediliyor...${NC}"

# ROS2 Humble (veya kurulu olan dağıtım)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}   ✓ ROS2 Humble${NC}"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo -e "${GREEN}   ✓ ROS2 Foxy${NC}"
else
    echo -e "${RED}   ✗ ROS2 dağıtımı bulunamadı!${NC}"
    exit 1
fi

# Workspace install source et
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
    echo -e "${GREEN}   ✓ Workspace: ${WS_DIR}${NC}"
else
    echo -e "${YELLOW}   ⚠ Workspace build edilmemiş. Derleniyor...${NC}"
    cd "${WS_DIR}"
    colcon build --packages-select teknofest_control
    source "${WS_DIR}/install/setup.bash"
fi

echo ""
echo -e "${YELLOW}🚀 Mock Target Tracking başlatılıyor...${NC}"
echo -e "${BLUE}   Mock Server URL: ${MOCK_URL}${NC}"
echo ""

# Launch dosyasını çalıştır
ros2 launch teknofest_control mock_tracking.launch.py \
    mock_server_url:="${MOCK_URL}" \
    enable_px4_bridge:=false \
    tracking_speed:=15.0 \
    lock_distance:=20.0

echo -e "${GREEN}✓ Sistem kapatıldı${NC}"
