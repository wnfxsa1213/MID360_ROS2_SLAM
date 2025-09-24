#include <rclcpp/rclcpp.hpp>
#include "point_cloud_filter_bridge.h"

int main(int argc, char** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建节点选项
    rclcpp::NodeOptions options;
    
    try {
        // 创建点云过滤桥接节点
        auto node = std::make_shared<point_cloud_filter::PointCloudFilterBridge>(options);
        
        RCLCPP_INFO(rclcpp::get_logger("main"), 
            "启动点云过滤桥接节点...");
        
        // 启动节点
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), 
            "节点运行时出现异常: %s", e.what());
        return -1;
    }
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "点云过滤桥接节点已安全关闭");
    
    return 0;
}