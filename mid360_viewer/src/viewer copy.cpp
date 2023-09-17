
#include <ros/ros.h

// livox custom msg header
#include <livox_ros_driver2/CustomMsg.h>

// ros pointcloud2 and PCL pointcloud header
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


void lidarCallbackCustom(const livox_ros_driver2::CustomMsgConstPtr &msg) {
    ROS_INFO("Custom msg callback.");
}


void lidarCallbackPointcloud2(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    ROS_INFO("Ros PointCloud2 callback.");
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "viewer");
    ros::NodeHandle nh;

    ROS_INFO("--> Start mid360 viewer.");


    int msg_type=0;
    nh.getParam("msg_type", msg_type);

    ROS_WARN_STREAM("==> Msg type: " << msg_type <<", " << ((msg_type==0) ? "custom " : "ros pointcloud2"));

    ros::Subscriber subPointCloud;
    if(msg_type == 0)
        subPointCloud = nh.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 100, &lidarCallbackCustom);
    else if(msg_type == 1)
        subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 100, &lidarCallbackPointcloud2);

    ros::Rate r(100);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
