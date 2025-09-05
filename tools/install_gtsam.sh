#!/bin/bash

# GTSAM库安装脚本
# 为MID360 SLAM项目安装GTSAM (Georgia Tech Smoothing and Mapping library)

set -e

echo "🔧 开始安装GTSAM库..."

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 检查系统
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$NAME
    VER=$VERSION_ID
fi

echo -e "${BLUE}检测到系统: $OS $VER${NC}"

# 1. 安装基础依赖
echo -e "${YELLOW}[1/5] 安装基础依赖...${NC}"
sudo apt-get update
sudo apt-get install -y \
    cmake \
    build-essential \
    libeigen3-dev \
    libboost-all-dev \
    libtbb-dev \
    git

# 2. 检查是否已有GTSAM
if pkg-config --exists gtsam; then
    echo -e "${GREEN}✅ GTSAM已安装，版本: $(pkg-config --modversion gtsam)${NC}"
    exit 0
fi

# 3. 创建构建目录
echo -e "${YELLOW}[2/5] 准备构建环境...${NC}"
TEMP_DIR="/tmp/gtsam_build"
rm -rf $TEMP_DIR
mkdir -p $TEMP_DIR
cd $TEMP_DIR

# 4. 克隆GTSAM源码
echo -e "${YELLOW}[3/5] 下载GTSAM源码...${NC}"
git clone https://github.com/borglab/gtsam.git
cd gtsam

# 5. 切换到稳定版本 (4.1.1)
echo -e "${YELLOW}切换到GTSAM 4.1.1版本...${NC}"
git checkout 4.1.1

# 6. 配置和编译
echo -e "${YELLOW}[4/5] 配置和编译GTSAM...${NC}"
mkdir build && cd build

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_DOCS=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V4=OFF

# 使用并行编译加速
CORES=$(nproc)
echo -e "${BLUE}使用 ${CORES} 个核心进行编译...${NC}"
make -j${CORES}

# 7. 安装
echo -e "${YELLOW}[5/5] 安装GTSAM...${NC}"
sudo make install
sudo ldconfig

# 8. 验证安装
echo -e "${YELLOW}验证安装...${NC}"
if pkg-config --exists gtsam; then
    GTSAM_VERSION=$(pkg-config --modversion gtsam)
    echo -e "${GREEN}🎉 GTSAM安装成功！版本: ${GTSAM_VERSION}${NC}"
    
    echo -e "${BLUE}安装路径信息:${NC}"
    echo "头文件: $(pkg-config --cflags gtsam)"
    echo "库文件: $(pkg-config --libs gtsam)"
    
    # 清理临时文件
    echo -e "${YELLOW}清理临时文件...${NC}"
    cd /
    rm -rf $TEMP_DIR
    
    echo -e "${GREEN}✅ GTSAM安装完成！现在可以编译pgo和hba包了。${NC}"
    echo -e "${YELLOW}下一步: 运行 './tools/slam_tools.sh build' 重新编译项目${NC}"
    
else
    echo -e "${RED}❌ GTSAM安装可能失败，请检查错误信息${NC}"
    exit 1
fi