#ifndef POINTCLOUD_ASSEMBLER_H
#define POINTCLOUD_ASSEMBLER_H

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <chrono>
#include <tf2/transform_datatypes.h>

#include "geometry_msgs/msg/transform_stamped.hpp"

class PointCloudAssembler : public rclcpp::Node
{
    public:
        PointCloudAssembler();
        ~PointCloudAssembler();

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_frontSubscriber;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_backSubscriber;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_mergePointCloudPub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_filterPointCloudPub;

        std::mutex m_transformMutex;
        //tf2_ros::Buffer tfBuffer;
        //tf2_ros::TransformListener tfListener;

        std::shared_ptr<tf2_ros::TransformListener> m_tfListener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;

        double m_filterMinX, m_filterMinY, m_filterMinZ;
        double m_filterMaxX, m_filterMaxY, m_filterMaxZ;

        std::string m_robotFrame, m_frontTopic, m_backTopic, m_mergeTopic, m_filterTopic;
        geometry_msgs::msg::TransformStamped m_transformFront, m_transformBack;

        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr frontPCL, const sensor_msgs::msg::PointCloud2::SharedPtr backPCL);
        void transformPointCloud(const std::string& targetFrame, const sensor_msgs::msg::PointCloud2::SharedPtr inputCloud, sensor_msgs::msg::PointCloud2& outputCloud);
        void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZ>& outputCloud);
};

#endif
