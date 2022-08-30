#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <vector>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

class Normal_Estimation{
    public:
    Normal_Estimation();

    private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_node_sub_;
    ros::Publisher ne_pub_;
    void ne_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg);
};

Normal_Estimation::Normal_Estimation() : nh_("~"){
    lidar_node_sub_ = nh_.subscribe("/Combined_velo", 10, &Normal_Estimation::ne_callback, this);
}

void Normal_Estimation::ne_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud_msg, *laserCloudIn);
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn_voxel (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> vox;
    double leaf_size = 0.05;
    vox.setInputCloud(laserCloudIn);
    vox.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox.filter(*laserCloudIn_voxel);

    // Normal Estimation
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    int K = 20;
    int num_pc = laserCloudIn_voxel->points.size();
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
    cloud_normal->points.reserve(num_pc);

    kdtree.setInputCloud(laserCloudIn_voxel);
    pcl::Normal normal_tmp;
    std::vector<int> idxes(K);
    std::vector<float> sqr_dists(K);

    for(int i=0; i<num_pc; ++i){
        pcl::PointXYZ &query = laserCloudIn_voxel->points[i];
        kdtree.nearestKSearch(query, K, idxes, sqr_dists);
        pcl::PointCloud<pcl::PointXYZ>::Ptr NN(new pcl::PointCloud<pcl::PointXYZ>);
        NN->reserve(num_pc);
        for(int tgt_idx: idxes){
            NN->points.emplace_back(laserCloudIn_voxel->points[tgt_idx]);
        }
        Eigen::Matrix3f cov;
        Eigen::Vector4f mean;
        pcl::computeMeanAndCovarianceMatrix(*NN, cov, mean);
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
        // svd에서 가장 작은 eigenvalue와 대응되는 eigenvector가 인근 pointcloud에서 추출한 normal vector

        Eigen::MatrixXf normal = (svd.matrixU().col(2));
        normal_tmp.normal_x = normal(0,0);
        normal_tmp.normal_y = normal(1,0);
        normal_tmp.normal_z = normal(2,0);
        cloud_normal->points.emplace_back(normal_tmp);
    } 
}

int main(int argc, char** argv){
    ros::init(argc, argv, "normal_estimation");
    Normal_Estimation node;
    ros::spin();
    return 0;
}

