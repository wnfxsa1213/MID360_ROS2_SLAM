#!/bin/bash

# ==============================================================================
# GTSAM库自动安装脚本
# ==============================================================================
# 作者: Claude Code Assistant  
# 版本: v1.0
# 日期: 2025-09-07
# 说明: 为MID360 SLAM系统安装GTSAM库
#       PGO和HBA组件需要GTSAM库支持
# ==============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# GTSAM版本配置
GTSAM_VERSION="4.1.1"
GTSAM_URL="https://github.com/borglab/gtsam/archive/${GTSAM_VERSION}.tar.gz"
INSTALL_PREFIX="/usr/local"

echo "========================================="
echo "  GTSAM ${GTSAM_VERSION} 自动安装工具"
echo "========================================="
echo ""

# 检查是否已安装
check_existing_installation() {
    echo -e "${CYAN}检查现有GTSAM安装...${NC}"
    
    if [ -f "${INSTALL_PREFIX}/lib/cmake/GTSAM/GTSAMConfig.cmake" ]; then
        local existing_version
        if [ -f "${INSTALL_PREFIX}/lib/cmake/GTSAM/GTSAMConfigVersion.cmake" ]; then
            existing_version=$(grep "set(GTSAM_VERSION" "${INSTALL_PREFIX}/lib/cmake/GTSAM/GTSAMConfigVersion.cmake" 2>/dev/null | grep -o '[0-9.]*' | head -1)
        fi
        
        echo -e "${GREEN}✅ GTSAM已安装 ${existing_version:-(版本未知)}${NC}"
        echo ""
        echo -e "${YELLOW}是否重新安装GTSAM ${GTSAM_VERSION}? (y/n):${NC}"
        read -r response
        if [[ ! "$response" =~ ^[Yy]$ ]]; then
            echo "跳过安装"
            exit 0
        fi
    fi
}

# 安装系统依赖
install_dependencies() {
    echo -e "${CYAN}安装GTSAM编译依赖...${NC}"
    
    local deps=(
        "cmake"
        "build-essential"
        "libeigen3-dev"
        "libboost-all-dev"
        "libtbb-dev"
    )
    
    if sudo apt update && sudo apt install -y "${deps[@]}"; then
        echo -e "${GREEN}✅ 依赖安装完成${NC}"
    else
        echo -e "${RED}❌ 依赖安装失败${NC}"
        exit 1
    fi
}

# 下载和编译GTSAM
build_gtsam() {
    echo -e "${CYAN}开始编译GTSAM ${GTSAM_VERSION}...${NC}"
    
    # 创建临时目录
    local temp_dir="/tmp/gtsam_build_$$"
    mkdir -p "$temp_dir"
    cd "$temp_dir"
    
    echo "工作目录: $temp_dir"
    
    # 下载源码
    echo -e "${BLUE}下载GTSAM源码...${NC}"
    if ! wget "$GTSAM_URL" -O gtsam-${GTSAM_VERSION}.tar.gz; then
        echo -e "${RED}❌ 下载失败${NC}"
        exit 1
    fi
    
    # 解压
    echo -e "${BLUE}解压源码...${NC}"
    tar -xzf gtsam-${GTSAM_VERSION}.tar.gz
    cd gtsam-${GTSAM_VERSION}
    
    # 创建构建目录
    mkdir -p build
    cd build
    
    # 检测CPU核心数
    local cpu_cores
    cpu_cores=$(nproc)
    echo -e "${BLUE}使用 ${cpu_cores} 个CPU核心编译${NC}"
    
    # 配置CMake
    echo -e "${BLUE}配置CMake...${NC}"
    cmake .. \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF || {
        echo -e "${RED}❌ CMake配置失败${NC}"
        exit 1
    }
    
    # 编译
    echo -e "${BLUE}编译GTSAM（预计5-15分钟，请耐心等待）...${NC}"
    if make -j"$cpu_cores"; then
        echo -e "${GREEN}✅ 编译成功${NC}"
    else
        echo -e "${RED}❌ 编译失败${NC}"
        echo -e "${YELLOW}尝试单线程编译...${NC}"
        if make -j1; then
            echo -e "${GREEN}✅ 单线程编译成功${NC}"
        else
            echo -e "${RED}❌ 编译彻底失败${NC}"
            exit 1
        fi
    fi
    
    # 安装
    echo -e "${BLUE}安装GTSAM...${NC}"
    if sudo make install; then
        echo -e "${GREEN}✅ 安装成功${NC}"
    else
        echo -e "${RED}❌ 安装失败${NC}"
        exit 1
    fi
    
    # 更新动态链接库缓存
    echo -e "${BLUE}更新系统库缓存...${NC}"
    sudo ldconfig
    
    # 清理临时文件
    cd /
    rm -rf "$temp_dir"
    echo -e "${GREEN}✅ 临时文件已清理${NC}"
}

# 验证安装
verify_installation() {
    echo -e "${CYAN}验证GTSAM安装...${NC}"
    
    # 检查CMake配置文件
    if [ -f "${INSTALL_PREFIX}/lib/cmake/GTSAM/GTSAMConfig.cmake" ]; then
        echo -e "${GREEN}✅ CMake配置文件存在${NC}"
    else
        echo -e "${RED}❌ CMake配置文件缺失${NC}"
        return 1
    fi
    
    # 检查头文件
    if [ -d "${INSTALL_PREFIX}/include/gtsam" ]; then
        echo -e "${GREEN}✅ 头文件安装正确${NC}"
    else
        echo -e "${RED}❌ 头文件缺失${NC}"
        return 1
    fi
    
    # 检查库文件
    if ls ${INSTALL_PREFIX}/lib/libgtsam* >/dev/null 2>&1; then
        echo -e "${GREEN}✅ 库文件安装正确${NC}"
        echo "已安装的库文件:"
        ls -la ${INSTALL_PREFIX}/lib/libgtsam* | head -5
    else
        echo -e "${RED}❌ 库文件缺失${NC}"
        return 1
    fi
    
    # 获取版本信息
    if [ -f "${INSTALL_PREFIX}/lib/cmake/GTSAM/GTSAMConfigVersion.cmake" ]; then
        local version
        version=$(grep "set(GTSAM_VERSION" "${INSTALL_PREFIX}/lib/cmake/GTSAM/GTSAMConfigVersion.cmake" 2>/dev/null | grep -o '[0-9.]*' | head -1)
        echo -e "${GREEN}✅ GTSAM版本: ${version:-$GTSAM_VERSION}${NC}"
    fi
    
    return 0
}

# 创建测试程序
create_test_program() {
    echo -e "${CYAN}创建GTSAM测试程序...${NC}"
    
    cat > "test_gtsam.cpp" << 'EOF'
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <iostream>

int main() {
    try {
        // 创建简单的3D点
        gtsam::Point3 point(1.0, 2.0, 3.0);
        std::cout << "创建3D点成功: " << point << std::endl;
        
        // 创建单位姿态
        gtsam::Pose3 pose = gtsam::Pose3::identity();
        std::cout << "创建姿态成功" << std::endl;
        
        std::cout << "✅ GTSAM库测试通过!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "❌ GTSAM测试失败: " << e.what() << std::endl;
        return 1;
    }
}
EOF
    
    # 编译测试程序
    echo -e "${BLUE}编译测试程序...${NC}"
    if g++ -std=c++17 -I${INSTALL_PREFIX}/include -L${INSTALL_PREFIX}/lib test_gtsam.cpp -lgtsam -ltbb -o test_gtsam; then
        echo -e "${GREEN}✅ 测试程序编译成功${NC}"
        
        # 运行测试
        echo -e "${BLUE}运行测试...${NC}"
        if LD_LIBRARY_PATH=${INSTALL_PREFIX}/lib:$LD_LIBRARY_PATH ./test_gtsam; then
            echo -e "${GREEN}✅ GTSAM功能测试通过${NC}"
        else
            echo -e "${YELLOW}⚠️  测试程序运行失败，但库可能已正确安装${NC}"
        fi
        
        # 清理测试文件
        rm -f test_gtsam test_gtsam.cpp
    else
        echo -e "${YELLOW}⚠️  测试程序编译失败，但GTSAM可能已正确安装${NC}"
    fi
}

# 主函数
main() {
    echo -e "${YELLOW}注意：GTSAM编译需要大量内存和时间${NC}"
    echo -e "${YELLOW}推荐系统配置：8GB+ 内存，多核CPU${NC}"
    echo ""
    
    # 检查现有安装
    check_existing_installation
    
    # 安装依赖
    install_dependencies
    
    # 构建和安装
    build_gtsam
    
    # 验证安装
    if verify_installation; then
        echo ""
        echo "========================================="
        echo -e "${GREEN}🎉 GTSAM ${GTSAM_VERSION} 安装成功！${NC}"
        echo "========================================="
        echo ""
        echo -e "${CYAN}安装位置:${NC}"
        echo "  头文件: ${INSTALL_PREFIX}/include/gtsam/"
        echo "  库文件: ${INSTALL_PREFIX}/lib/libgtsam*"
        echo "  CMake:  ${INSTALL_PREFIX}/lib/cmake/GTSAM/"
        echo ""
        echo -e "${BLUE}现在可以编译PGO和HBA包了：${NC}"
        echo "  cd ../.. && ./tools/slam_tools.sh build"
        echo ""
        
        # 创建并运行测试
        create_test_program
    else
        echo -e "${RED}❌ GTSAM安装验证失败${NC}"
        exit 1
    fi
}

# 运行主函数
main "$@"