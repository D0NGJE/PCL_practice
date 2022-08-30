#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <cmath>
#include <stdio.h>

class Clustering{
    public:
    Clustering();

    private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_node_sub_;

    void clustering_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg);
};

Clustering::Clustering() : nh_("~"){
    lidar_node_sub_ = nh_.subscribe("/Combined_velo", 10, &Clustering::clustering_callback, this);
}

void Clustering::clustering_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_cloud_msg, *laserCloudIn);

    // Create the filtering Object : down sample the dataset using a leaf size of 0.01m(1cm)
    pcl::VoxelGrid<pcl::PointXYZI> vox;
    double leafsize = 0.01;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    vox.setInputCloud(laserCloudIn);
    vox.setLeafSize(leafsize, leafsize, leafsize);
    vox.filter(*cloud_filtered);

    pcl::PassThrough<pcl::PointXYZI> pass_i;
    pass_i.setInputCloud(cloud_filtered);
    pass_i.setFilterFieldName("intensity");
    pass_i.setFilterLimits(60,200);
    pass_i.filter(*cloud_filtered);

    // Euclidean Clustering
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_filtered);
    std::vector<pcl::PointIndices> cluster_indices; // 클러스터링 결과물의 index저장
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(cloud_filtered); // 입력 cloud
    ec.setClusterTolerance(0.3);
    ec.setMinClusterSize(2); // 최소 포인트 수
    ec.setMaxClusterSize(70); // 최대 포인트 수
    ec.setSearchMethod(tree); // 탐색 방법 지정
    ec.extract(cluster_indices); // 클러스터링 적용

    pcl::PointCloud<pcl::PointXYZI>::Ptr euclidean_clustered (new pcl::PointCloud<pcl::PointXYZI>);
    int j =0;
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
        for(const auto& idx : it->indices){
            euclidean_clustered->push_back((*cloud_filtered)[idx]);
        }
        j++;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "clustering");
    Clustering node;
    ros::spin();
    return 0;
}

