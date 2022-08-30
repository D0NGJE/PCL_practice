#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

class Filter_prac{
    public:
    Filter_prac();

    private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_node_sub_;
    ros::Publisher passthrough_pub_;
    ros::Publisher voxel_pub_;
    ros::Publisher sor_pub_;

    void filter_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg);
};

Filter_prac::Filter_prac() : nh_("~"){
    lidar_node_sub_ = nh_.subscribe("/Combined_velo", 10, &Filter_prac::filter_callback, this);
    passthrough_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/passthrough_points", 10);
    voxel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/voxel_points",10);
    sor_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/statistical_outlier_removal_points", 10);
}

void Filter_prac::filter_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_cloud_msg, *laserCloudIn);
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn_pass(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*laserCloudIn,*laserCloudIn_pass);
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn_voxel(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*laserCloudIn,*laserCloudIn_voxel);
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn_statistical(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*laserCloudIn,*laserCloudIn_statistical);

    // PassThrough filter (ROI)
    pcl::PassThrough<pcl::PointXYZI> pass_z;
    pass_z.setInputCloud(laserCloudIn_pass);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-4.0, -1.5);
    pass_z.filter(*laserCloudIn_pass);

    pcl::PassThrough<pcl::PointXYZI> pass_y;
    pass_y.setInputCloud(laserCloudIn_pass);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-3.5, 6.5);
    pass_y.filter(*laserCloudIn_pass);

    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pass_x.setInputCloud(laserCloudIn_pass);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(5,25);
    pass_x.filter(*laserCloudIn_pass);

    pcl::PassThrough<pcl::PointXYZI> pass_i;
    pass_i.setInputCloud(laserCloudIn_pass);
    pass_i.setFilterFieldName("intensity");
    pass_i.setFilterLimits(50, 200);
    pass_i.filter(*laserCloudIn_pass);

    sensor_msgs::PointCloud2 passthrough_msg;
    pcl::toROSMsg(*laserCloudIn_pass, passthrough_msg);
    passthrough_msg.header.frame_id = laserCloudIn->header.frame_id;
    passthrough_pub_.publish(passthrough_msg);

    // Voxel Grid filter (Down Sampling)
    double voxel_size = 0.8;
    pcl::VoxelGrid<pcl::PointXYZI> vox;
    vox.setInputCloud(laserCloudIn_voxel);
    vox.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox.filter(*laserCloudIn_voxel);

    sensor_msgs::PointCloud2 voxel_msg;
    pcl::toROSMsg(*laserCloudIn_voxel, voxel_msg);
    voxel_msg.header.frame_id = laserCloudIn->header.frame_id;
    voxel_pub_.publish(voxel_msg);

    // Statistical Outlier Removal
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_statistical(new pcl::PointCloud<pcl::PointXYZI>);
    int num_neighbor_points = 10;
    double std_multiplier = 1.0;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(laserCloudIn_statistical);
    sor.setMeanK(num_neighbor_points);
    sor.setStddevMulThresh(std_multiplier);
    sor.filter(*laserCloudIn_statistical);

    sensor_msgs::PointCloud2 statistical_msg;
    pcl::toROSMsg(*laserCloudIn_statistical, statistical_msg);
    statistical_msg.header.frame_id = laserCloudIn->header.frame_id;
    sor_pub_.publish(statistical_msg);
}



int main(int argc, char **argv){
    ros::init(argc, argv, "filter");
    Filter_prac node;
    ros::spin();
    return 0;
}
