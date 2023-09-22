
#include <string>
#include <vector>

#include <ros/ros.h>

// livox custom msg header
#include <livox_ros_driver2/CustomMsg.h>

// ros pointcloud2 and PCL pointcloud header
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define LIDAR_FREQUENCY (10)
#define PI (3.1415926f)
#define OUTPUT_SCAN_LINE (32)         // how many line output.

// FOV bottom to top
const double fov_bottom = -8;
const double fov_top = 54;

ros::Publisher pubNewPointCloud;
int g_printPoints = false;

void lidarCallbackCustom(const livox_ros_driver2::CustomMsgConstPtr &msg) {
    // ROS_INFO_STREAM("Custom msg callback. Size: " << msg->point_num);
    typedef std::pair<int, double> my_pair;
    std::vector<my_pair> index_yaw_pair;                // pair <point index in msg, yaw angle>

    // first sort by yaw  angle
    for(int i=0; i<msg->point_num; ++i){
        auto p = msg->points[i];
        if(abs(p.x)<0.01 || abs(p.y) < 0.01)            // skip too-near points.
            continue;
        double yaw = atan2(p.y, p.x);                   // angle 0 start from x+
        index_yaw_pair.push_back(my_pair(i, yaw));
    }
    sort(index_yaw_pair.begin(), index_yaw_pair.end(), [](const my_pair& p1, const my_pair& p2){    // sort by yaw-angle
        return p1.second < p2.second;
    });

    // calcualte ring and offset-time, and save to intensity;
    double rad_to_deg = 180/PI;
    const double vertical_angle_resolution = (fov_top - fov_bottom)/OUTPUT_SCAN_LINE;
    pcl::PointCloud<pcl::PointXYZI> pc_new;
    for(const my_pair& idx_yaw : index_yaw_pair){
        auto p = msg->points[idx_yaw.first];

        // calculate pitch angle.
        double pitch = atan2(p.z, sqrt(p.x*p.x+p.y*p.y))*rad_to_deg;
        int line = int((pitch - fov_bottom) / vertical_angle_resolution);
        assert(line>=0 && line<=OUTPUT_SCAN_LINE);

        pcl::PointXYZI p_new;
        p_new.x = p.x;
        p_new.y = p.y;
        p_new.z = p.z;
        p_new.intensity = line + float(p.offset_time)*1e-9*LIDAR_FREQUENCY;
        // ROS_INFO_STREAM("line: "<< line <<", offset_time" << p.offset_time <<", intensity: " << p_new.intensity);
        pc_new.points.push_back(p_new);
    }
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(pc_new, pc_msg);
    pc_msg.header = msg->header;
    pubNewPointCloud.publish(pc_msg);

    // test codes;
    if(g_printPoints){
        ROS_WARN("==> Output points with ring, offset, and angles ======================");
        for(int i=0; i<pc_new.points.size(); ++i){
            pcl::PointXYZI p = pc_new.points[i];
            ROS_INFO_STREAM("Point: " << i
                                    << ", XYZ [" << p.x << ", " << p.y << ", " << p.z
                                    << "], x-y plane angle: " << atan2(p.y, p.x) * 180 / PI
                                    << ", Ring: " << int(p.intensity)
                                    << ", offset-scale: " << p.intensity - int(p.intensity));
        }
    }
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "converter");
    ros::NodeHandle nh("~");
    ROS_INFO("--> Start mid360 msg type converter.");
    nh.getParam("print_points", g_printPoints);

    ros::Subscriber subPointCloud = nh.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 100, &lidarCallbackCustom);
    pubNewPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/ros_pc_with_intensity", 10);
    ros::Rate r(100);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
