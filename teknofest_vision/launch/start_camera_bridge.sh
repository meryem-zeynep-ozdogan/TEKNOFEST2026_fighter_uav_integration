#!/bin/bash
# =====================================================================
# TEKNOFEST Camera Bridge Script
# Hero uçağının kamerasını ROS2'ye aktarır
# =====================================================================

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# Gazebo topic (model spawn edildikten sonra bu isimle oluşur)
GZ_TOPIC="/world/competition/model/rc_cessna_0/link/base_link/sensor/front_camera/image"

echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  TEKNOFEST Camera Bridge - HERO (rc_cessna_0)${NC}"
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}Gazebo Topic:${NC} $GZ_TOPIC"
echo -e "${YELLOW}ROS2 Topic:${NC}   /hero/camera/image_raw"
echo ""

# Gazebo topic'in var olup olmadığını kontrol et
echo -e "${YELLOW}Gazebo topic kontrol ediliyor...${NC}"
if gz topic -l 2>/dev/null | grep -q "front_camera/image"; then
    echo -e "${GREEN}✓ Kamera topic'i bulundu!${NC}"
else
    echo -e "${YELLOW}! Kamera topic'i henüz yok. Simülasyon çalışıyor mu?${NC}"
    echo "  Kontrol: gz topic -l | grep image"
fi
echo ""

# ROS2 bridge başlat
echo -e "${YELLOW}ROS2 bridge başlatılıyor...${NC}"
echo -e "${CYAN}Durdurmak için: Ctrl+C${NC}"
echo ""

# ros_gz_image bridge
ros2 run ros_gz_image image_bridge "$GZ_TOPIC"
