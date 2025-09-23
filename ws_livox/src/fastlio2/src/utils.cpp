#include "utils.h"
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = msg->point_num;

    // 预先计算范围平方，避免重复计算
    const double min_range_sq = min_range * min_range;
    const double max_range_sq = max_range * max_range;

    // 更准确的内存预分配：基于过滤比例和有效点估算
    int estimated_points = point_num / filter_num;
    cloud->reserve(estimated_points + 100); // 添加小缓冲避免重新分配

    // 预分配临时点以减少栈分配开销
    pcl::PointXYZINormal temp_point;

    for (int i = 0; i < point_num; i += filter_num)
    {
        const auto& livox_point = msg->points[i];
        if ((livox_point.line < 4) && ((livox_point.tag & 0x30) == 0x10 || (livox_point.tag & 0x30) == 0x00))
        {
            float x = livox_point.x;
            float y = livox_point.y;
            float z = livox_point.z;

            // 距离检查优化：先计算平方距离
            double dist_sq = x * x + y * y + z * z;
            if (dist_sq < min_range_sq || dist_sq > max_range_sq)
                continue;

            // 直接在预分配的点对象上设置值，避免重复构造
            temp_point.x = x;
            temp_point.y = y;
            temp_point.z = z;
            temp_point.intensity = livox_point.reflectivity;
            temp_point.curvature = livox_point.offset_time / 1000000.0f;

            cloud->push_back(temp_point);
        }
    }

    // PCL PointCloud不支持shrink_to_fit，使用points向量的优化
    cloud->points.shrink_to_fit();
    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
