#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <tf/transform_datatypes.h>

class LidarMapping {
public:
    LidarMapping() {
        
        ros::NodeHandle nh;
        lidar_sub = nh.subscribe("/airsim/lidar/point_cloud/Lidar_Avia_front", 1, &LidarMapping::lidarCallback, this);
        pose_sub = nh.subscribe("/mavros/local_position/pose", 1, &LidarMapping::poseCallback, this);
        map_pub = nh.advertise<octomap_msgs::Octomap>("/octomap_full", 1);
        path_pub = nh.advertise<nav_msgs::Path>("/trajectory", 1);
        octree = new octomap::OcTree(0.1); // 10cm resolution
        path.header.frame_id = "map";
    }

    ~LidarMapping() {
        delete octree;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose = *msg;
        updatePath();
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        octomap::Pointcloud octoCloud;
        octomap::pointCloud2ToOctomap(*cloud_msg, octoCloud);
        octomap::point3d sensorOrigin(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
        octree->insertPointCloud(octoCloud, sensorOrigin);
        publishOctomap();
    }

    void updatePath() {
        path.poses.push_back(current_pose);
        path.header.stamp = ros::Time::now();
        path_pub.publish(path);
    }

    void publishOctomap() {
        octomap_msgs::Octomap map_msg;
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = ros::Time::now();
        if (octomap_msgs::binaryMapToMsg(*octree, map_msg)) {
            map_pub.publish(map_msg);
        }
    }

private:
    ros::Subscriber lidar_sub;
    ros::Subscriber pose_sub;
    ros::Publisher map_pub;
    ros::Publisher path_pub;
    geometry_msgs::PoseStamped current_pose;
    nav_msgs::Path path;
    octomap::OcTree* octree;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_mapping_node");
    LidarMapping mapper;
    ros::spin();
    return 0;
}
