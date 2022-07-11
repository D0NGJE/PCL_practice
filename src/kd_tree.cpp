#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <ctime>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

class KD_tree_prac{
    public:
    KD_tree_prac();

    private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_node_sub_;
    ros::Publisher KNN_pub_;
    ros::Publisher Radius_pub_1_;
    ros::Publisher Radius_pub_2_;

    void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg);
};

KD_tree_prac::KD_tree_prac() : nh_("~"){
    lidar_node_sub_ = nh_.subscribe("/Combined_velo", 10, &KD_tree_prac::lidar_callback, this); // lidar_data
    KNN_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/KNN_points", 10);
    Radius_pub_1_ = nh_.advertise<sensor_msgs::PointCloud2>("/Radius_points_1", 10);
    Radius_pub_2_ = nh_.advertise<sensor_msgs::PointCloud2>("/Radius_points_2", 10);
}

void KD_tree_prac::lidar_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud_msg, *laserCloudIn);

    // Radius Search
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    int radius = 8.0;
    std::vector<int> idxes;
    std::vector<float> sqr_dists;

    kdtree.setInputCloud(laserCloudIn);
    pcl::PointXYZ query1(0.0, 0.0, 0.0); //기준점
    pcl::PointXYZ query2(20.0, 15.0, 0.0);
    // input : query point, radius
    // output : indices, squred distance

    kdtree.radiusSearch(query1, radius, idxes, sqr_dists);
    for(const auto& idx: idxes){
        boundary1 -> points.push_back(laserCloudIn->points[idx]);
    }

    kdtree.radiusSearch(query2, radius, idxes, sqr_dists);
    for(const auto& idx: idxes){
        boundary2 -> points.push_back(laserCloudIn->points[idx]);
    }

    sensor_msgs::PointCloud2 query1_msg;
    pcl::toROSMsg(*boundary1, query1_msg);
    query1_msg.header.frame_id = laserCloudIn->header.frame_id;
    Radius_pub_1_.publish(query1_msg);

    sensor_msgs::PointCloud2 query2_msg;
    pcl::toROSMsg(*boundary2, query2_msg);
    query2_msg.header.frame_id = laserCloudIn->header.frame_id;
    Radius_pub_2_.publish(query2_msg);

    // K-nearest Neighbor Search
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary3(new pcl::PointCloud<pcl::PointXYZ>);
    int N = 1000;
    pcl::PointXYZ query3(3.0, 3.0, 0.0);
    // input : query point, N
    // output : indices, squred distances

    kdtree.nearestKSearch(query3, N, idxes, sqr_dists);
    for(const auto& idx: idxes){
        boundary3->points.push_back(laserCloudIn->points[idx]);
    }

    sensor_msgs::PointCloud2 knn_msg;
    pcl::toROSMsg(*boundary3, knn_msg);
    knn_msg.header.frame_id = laserCloudIn->header.frame_id;
    KNN_pub_.publish(knn_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "kd_tree");
    KD_tree_prac node;
    ros::spin();
    return 0;
}
