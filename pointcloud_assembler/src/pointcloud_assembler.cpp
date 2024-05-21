#include "pointcloud_assembler/pointcloud_assembler.hpp"

PointCloudAssembler::PointCloudAssembler()
    : Node("pointcloud_assembler")
{
    RCLCPP_INFO(this->get_logger(),"Point Cloud Assembler Module Is Online");

    // Declare parameters
    RCLCPP_INFO(this->get_logger(),"Declaring The Parameters");
    this->declare_parameter<std::string>("robot_frame", "/base_footprint");
    this->declare_parameter<std::string>("front_topic", "/front_cloud");
    this->declare_parameter<std::string>("back_topic", "/back_cloud");
    this->declare_parameter<std::string>("merge_topic", "/merged_cloud");
    this->declare_parameter<std::string>("filter_topic", "/filtered_cloud");

    this->declare_parameter<double>("filter_min_x", 0.0);
    this->declare_parameter<double>("filter_min_y", 0.0);
    this->declare_parameter<double>("filter_min_z", 0.0);
    this->declare_parameter<double>("filter_max_x", 10.0);
    this->declare_parameter<double>("filter_max_y", 10.0);
    this->declare_parameter<double>("filter_max_z", 10.0);

    //Get Parameters
    RCLCPP_INFO(this->get_logger(),"Getting The Parameters");
    this->get_parameter("robot_frame", m_robotFrame);
    this->get_parameter("front_topic", m_frontTopic);
    this->get_parameter("back_topic", m_backTopic);
    this->get_parameter("merge_topic", m_mergeTopic);
    this->get_parameter("filter_topic", m_filterTopic);

    this->get_parameter("filter_min_x", m_filterMinX);
    this->get_parameter("filter_min_y", m_filterMinY);
    this->get_parameter("filter_min_z", m_filterMinZ);
    this->get_parameter("filter_max_x", m_filterMaxX);
    this->get_parameter("filter_max_y", m_filterMaxY);
    this->get_parameter("filter_max_z", m_filterMaxZ);

    m_tfBuffer =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tfListener =
      std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

    // Setup subscribers and publishers
    // // Setup subscribers and publishers
    // frontSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     frontTopic, 1, std::bind(&PointCloudAssembler::pointCloudCallback, this, std::placeholders::_1, nullptr));

    // backSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    // backTopic, 1, std::bind(&PointCloudAssembler::pointCloudCallback, this, nullptr, std::placeholders::_1));

    // Setting Up The Subscribers  publishers
    m_frontSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        m_frontTopic, 1,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr frontPCL)
        {
            pointCloudCallback(frontPCL, nullptr);
        });

    m_backSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        m_backTopic, 1,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr backPCL)
        {
            pointCloudCallback(nullptr, backPCL);
        });

    // Setting Up The Publishers
    m_mergePointCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_mergeTopic, 1);
    m_filterPointCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_filterTopic, 1);
}

PointCloudAssembler::~PointCloudAssembler()
{
    RCLCPP_INFO(this->get_logger(),"Point Cloud Assembler Module Is Offline");
}

void PointCloudAssembler::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr frontPCL, const sensor_msgs::msg::PointCloud2::SharedPtr backPCL)
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Point Cloud Callback Invoked");
    
    try
    {
        m_transformFront = m_tfBuffer->lookupTransform(
        m_robotFrame, frontPCL->header.frame_id, tf2::TimePointZero);

        m_transformBack = m_tfBuffer->lookupTransform(
        m_robotFrame, backPCL->header.frame_id, tf2::TimePointZero);

        sensor_msgs::msg::PointCloud2 frontCloudTransformed, backCloudTransformed;

        pcl_ros::transformPointCloud(m_robotFrame, m_transformFront, *frontPCL, frontCloudTransformed);
        pcl_ros::transformPointCloud(m_robotFrame, m_transformBack, *backPCL, backCloudTransformed);

        frontCloudTransformed.header.frame_id = m_robotFrame;
        backCloudTransformed.header.frame_id = m_robotFrame;

        //Creating pcl data structures
        pcl::PointCloud<pcl::PointXYZ>::Ptr frontPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr backPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr mergePointCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> ::Ptr croppedPointCloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(frontCloudTransformed, *frontPointCloud);
        pcl::fromROSMsg(backCloudTransformed, *backPointCloud);

        // Add all pointclouds
        *mergePointCloud += *frontPointCloud;
        *mergePointCloud += *backPointCloud;

        sensor_msgs::msg::PointCloud2 merge_cloud_msg;
        pcl::toROSMsg(*mergePointCloud, merge_cloud_msg);
        merge_cloud_msg.header.frame_id = m_robotFrame;
        m_mergePointCloudPub->publish(merge_cloud_msg);

        pcl::CropBox<pcl::PointXYZ> cropBox;
        cropBox.setInputCloud(mergePointCloud);

        Eigen::Vector4f min_point = Eigen::Vector4f(m_filterMinX, m_filterMinY, m_filterMinZ, 0);
        Eigen::Vector4f max_point = Eigen::Vector4f(m_filterMaxX, m_filterMaxY, m_filterMaxZ, 0);
        cropBox.setMin(min_point);
        cropBox.setMax(max_point);
        cropBox.filter(*croppedPointCloud);

        sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*croppedPointCloud, filtered_cloud_msg);
        filtered_cloud_msg.header.frame_id = m_robotFrame;
        m_filterPointCloudPub->publish(filtered_cloud_msg);
    }
    catch(const tf2::TransformException& ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Transform exception : %s", ex.what());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudAssembler>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

