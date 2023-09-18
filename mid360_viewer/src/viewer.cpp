
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

#define PI (3.1415926)
#define SCAN_LINE (4)
#define LIDAR_FREQUENCY (10)

ros::Publisher pubRosPointCloud2, pubScanLines[SCAN_LINE], pubLine0Time;
int g_printPoints = false;


void lidarCallbackCustom(const livox_ros_driver2::CustomMsgConstPtr &msg) {
    ROS_INFO_STREAM("Custom msg callback. Size: " << msg->point_num);
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> v_pc;
    for(int i=0; i<SCAN_LINE; ++i){
        v_pc.push_back(pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>()));
    }
    for(auto p : msg->points){
        pcl::PointXYZINormal p_new;
        if(abs(p.x)<0.01 || abs(p.y) < 0.01)         // skip too-near points.
            continue;

        // convert ros msg's point to pcl format.
        p_new.x = p.x;
        p_new.y = p.y;
        p_new.z = p.z;
        // calculate time-offset and save in `intensity`
        p_new.intensity = p.offset_time*1e-9*LIDAR_FREQUENCY; // offset_time: time relative to the base time (us)
        // calculate the angle of each points and save in `normal.z`
        double distance = sqrt(p.x*p.x+p.y*p.y);
        double angle = atan2(p.z, distance);
        p_new.normal_x = p.line;
        p_new.normal_z = angle*180/PI;      // angle in degree.

        // save points to v_pc;
        v_pc[p.line]->points.push_back(p_new);
    }
    for(int i=0; i<SCAN_LINE; ++i){
        sensor_msgs::PointCloud2 pcMsg;
        pcl::toROSMsg(*v_pc[i], pcMsg);
        pcMsg.header = msg->header;
        pcMsg.header.frame_id = "map";
        pubScanLines[i].publish(pcMsg);
    }

    // print time-offset and anble
    if(g_printPoints){
        // for(int i=0; i<v_pc[0]->points.size(); ++i){
        //     auto p = v_pc[0]->points[i];
        //     ROS_INFO_STREAM("i=" << i << ", offset: " << p.intensity << ", line: " << p.normal_x << ", angle: " << p.normal_z);
        // }
        for(int i=0; i<msg->point_num; ++i){
            auto p = msg->points[i];
            if(abs(p.x)<0.01 || abs(p.y) < 0.01)         // skip too-near points.
                continue;
            double distance = sqrt(p.x*p.x+p.y*p.y);
            double angle = atan2(p.z, distance)*180/PI;
            ROS_INFO_STREAM("i=" << i << ", offset: " << p.offset_time << ", line: " << (int)(p.line) << ", angle: " << angle);
        }
    }

    // sort points by angles
    // std::vector<std::pair<double, std::pair<int, int> > > angles;       // vector( angle, <index, line> )
    // for(int i=0; i<msg->point_num; ++i){
    //     auto p = msg->points[i];
    //     if(abs(p.x)<0.01 || abs(p.y) < 0.01)         // skip too-near points.
    //         continue;
    //     double distance = sqrt(p.x*p.x+p.y*p.y);
    //     double angle = atan2(p.z, distance)*180/PI;
    //     auto index_line = std::pair<int, int>(i, p.line);
    //     angles.push_back(std::pair<double, std::pair<int,int>> (angle, index_line));
    // }
    // sort(angles.begin(), angles.end(), [](std::pair<double, std::pair<int,int>> p1, std::pair<double, std::pair<int,int>> p2){
    //     return p1.first < p2.first;
    // });
    // ROS_ERROR_STREAM("========================");
    // for(auto p : angles){
    //     ROS_INFO_STREAM("Angle: " << p.first <<", index: " << p.second.first <<", line: " << p.second.second);
    // }
}


void lidarCallbackPointcloud2(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *pc);

    ROS_INFO_STREAM("Ros PointCloud2 callback. Size: " << pc->points.size());
    
    // TODO: add ring calculation
    for(auto &p:pc->points){
        double distance = sqrt(p.x*p.x+p.y*p.y);
        double angle = atan2(p.z, distance);
        p.intensity = (angle+PI)/(2*PI);        // normalize to 0~1.
    }
    // convert pcl pointcloud to ros message
    sensor_msgs::PointCloud2 pcMsg;
    pcl::toROSMsg(*pc, pcMsg);
    pcMsg.header = msg->header;
    pcMsg.header.frame_id = "map";
    pubRosPointCloud2.publish(pcMsg);
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "viewer");
    ros::NodeHandle nh("~");

    ROS_INFO("--> Start mid360 viewer.");


    int msg_type=0;
    nh.getParam("msg_type", msg_type);
    nh.getParam("print_points", g_printPoints);

    ROS_WARN_STREAM("==> Msg type: " << msg_type <<", " << ((msg_type==0) ? "custom " : "ros pointcloud2"));

    // subscriber
    ros::Subscriber subPointCloud;
    if(msg_type == 0)
        subPointCloud = nh.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 100, &lidarCallbackCustom);
    else if(msg_type == 1)
        subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 100, &lidarCallbackPointcloud2);

    // publisher
    pubRosPointCloud2 = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_with_intensity", 10);
    for(int i=0; i<SCAN_LINE; ++i)
        pubScanLines[i] = nh.advertise<sensor_msgs::PointCloud2>("/lidar/line"+std::to_string(i), 10);
    // pubLine0Time = nh.advertise<sensor_msgs::PointCloud2>("/lidar/line0_time", 10);

    ros::Rate r(100);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
