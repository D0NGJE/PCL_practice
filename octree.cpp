#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <vector>
#include <ctime>

class Octree{
    public:
    Octree();

    private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_node_sub_;
    ros::Publisher octree_knn_pub_;
    ros::Publisher octree_radius_pub_;

    void octree_callback(const sensor_msgs::PointCloud2ConstPtr &input_lidar_msg);
};

Octree::Octree() : nh_("~"){
    lidar_node_sub_ = nh_.subscribe("/Combined_velo", 10, &Octree::octree_callback, this);
    octree_knn_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/octree_knn", 10);
    octree_radius_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/octree_radius", 10);
}

void Octree::octree_callback(const sensor_msgs::PointCloud2ConstPtr &input_lidar_msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_lidar_msg, *laserCloudIn);

    // create octree
    float resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud(laserCloudIn);
    octree.addPointsFromInputCloud();

    pcl::PointXYZ searchPoint(10.0, 10.0, 0); // searchpoint = ( , , ) 이따구로 넣으면 망함

    // Neighbors within voxel search
    std::vector<int> pointIdxVec;

    // if(octree.voxelSearch(searchPoint, pointIdxVec)){
    //     std::cout << "Neighbors within voxel search at (" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << ")" << std::endl;
    //     for(std::size_t i = 0; i < pointIdxVec.size(); ++i){
    //         std::cout << "   " << (*laserCloudIn)[pointIdxVec[i]].x
    //             << " " << (*laserCloudIn)[pointIdxVec[i]].y
    //             << " " << (*laserCloudIn)[pointIdxVec[i]].z << std::endl;
    //     }
    // }

    // K nearest neighbor search
    int K = 10;
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_knn (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> pointIdxKNNSearch;
    std::vector<float> pointKNNSquaredDistance;
    if(octree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0){
        for(std::size_t i =0; i<pointIdxKNNSearch.size(); ++i){
            boundary_knn->push_back((*laserCloudIn)[pointIdxKNNSearch[i]]);
        }
    }

    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = 3.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_radius (new pcl::PointCloud<pcl::PointXYZ>);
    if(octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
        for(std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
            boundary_radius->push_back((*laserCloudIn)[pointIdxRadiusSearch[i]]);
        }
    }

    sensor_msgs::PointCloud2 octree_knn_msg;
    pcl::toROSMsg(*boundary_knn, octree_knn_msg);
    octree_knn_msg.header.frame_id = input_lidar_msg->header.frame_id;
    octree_knn_pub_.publish(octree_knn_msg);

    sensor_msgs::PointCloud2 octree_radius_msg;
    pcl::toROSMsg(*boundary_radius, octree_radius_msg);
    octree_radius_msg.header.frame_id = input_lidar_msg->header.frame_id;
    octree_radius_pub_.publish(octree_radius_msg);
}


int main(int argc, char**argv){
    ros::init(argc, argv, "octree");
    Octree node;
    ros::spin();
    return 0;
}

