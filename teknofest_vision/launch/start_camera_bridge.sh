#!/bin/bash
# =====================================================================
# TEKNOFEST Camera Bridge Script
# Hero uçağının kamerasını ROS2'ye aktarır
#
# KULLANIM:
#   ./start_camera_bridge.sh              # Otomatik model bul
#   ./start_camera_bridge.sh hero_cessna  # Manuel model ismi
# =====================================================================

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  TEKNOFEST Camera Bridge${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo ""

# =====================================================================
# MODEL İSMİNİ BUL
# =====================================================================
if [ -n "$1" ]; then
    # Manuel model ismi verilmiş
    MODEL_NAME="$1"
    echo -e "${YELLOW}Manuel model ismi:${NC} $MODEL_NAME"
else
    # Otomatik bul - Gazebo topic listesinden kamera olan modeli bul
    echo -e "${YELLOW}Gazebo'da kamera topic'i aranıyor...${NC}"
    
    # Önce hero_cessna ara
    GZ_TOPIC=$(gz topic -l 2>/dev/null | grep "front_camera/image" | head -1)
    
    if [ -z "$GZ_TOPIC" ]; then
        echo -e "${RED}HATA: Kamera topic'i bulunamadı!${NC}"
        echo ""
        echo "Olası nedenler:"
        echo "  1. Gazebo simülasyonu çalışmıyor"
        echo "  2. Uçak henüz spawn edilmedi"
        echo "  3. Kamera sensörü tanımlı değil"
        echo ""
        echo "Kontrol komutları:"
        echo "  gz topic -l | grep image"
        echo "  gz model --list"
        echo ""
        exit 1
    fi
    
    # Topic'ten model ismini çıkar
    # Format: /world/competition/model/MODEL_NAME/link/base_link/sensor/front_camera/image
    MODEL_NAME=$(echo "$GZ_TOPIC" | sed -n 's|.*/model/\([^/]*\)/link.*|\1|p')
    echo -e "${GREEN}✓ Model bulundu:${NC} $MODEL_NAME"
fi

# Topic'i oluştur (yeni format: rc_cessna_0 üzerindeki front_camera)
GZ_TOPIC="/world/competition/model/${MODEL_NAME}/link/base_link/sensor/front_camera/image"
ROS_TOPIC="/${MODEL_NAME}/camera/image_raw"

echo ""
echo -e "${YELLOW}Gazebo Topic:${NC} $GZ_TOPIC"
echo -e "${YELLOW}ROS2 Topic:${NC}   $ROS_TOPIC"
echo ""

# Topic kontrolü
if ! gz topic -l 2>/dev/null | grep -q "$GZ_TOPIC"; then
    echo -e "${RED}UYARI: Topic bulunamadı!${NC}"
    echo "Mevcut kamera topic'leri:"
    gz topic -l 2>/dev/null | grep "front_camera" || echo "  (hiç yok)"
    echo ""
    echo "Yine de devam ediliyor..."
fi

echo -e "${CYAN}ROS2 bridge başlatılıyor...${NC}"
echo -e "${YELLOW}Durdurmak için: Ctrl+C${NC}"
echo ""

# ROS2 bridge başlat
ros2 run ros_gz_image image_bridge "$GZ_TOPIC" --ros-args -r "$GZ_TOPIC:=$ROS_TOPIC"
