#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef struct Point_{
    float x,y,z;
    int clusterID;
}Point;

class DBSCAN{
    public:
    DBSCAN();

    private:
    ros::NodeHandle nh_;
    ros::Subscriber non_ground_sub_;
    ros::Publisher dbscan_pub_;

    int minPoints_ = 8;
    double epsilon_ = (1.5*1.5);

    std::vector<Point> points_;

    void convertPoint(pcl::PointCloud<pcl::PointXYZ> laserCloudIn);
    std::vector<int> calculateCluster(Point point);
    int expandCluster(Point point, int clusterID);
    double calculateDistance(const Point& pointCore, const Point& pointTarget);
    void print_result();
    void dbscan_callback(const sensor_msgs::PointCloud2ConstPtr &input_non_ground_msg);
};

DBSCAN::DBSCAN() : nh_ ("~"){
    non_ground_sub_ = nh_.subscribe("/no_ground_points", 1, &DBSCAN::dbscan_callback, this);
    dbscan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dbscan_points", 1);
}

void DBSCAN::convertPoint(pcl::PointCloud<pcl::PointXYZ> laserCloudIn){
    // unsigned int i = 0;
    unsigned int num_points = laserCloudIn.points.size();
    Point *p = (Point *)calloc(num_points, sizeof(Point));
    for(int i = 0; i < laserCloudIn.points.size(); i++){
        p[i].x = laserCloudIn.points[i].x;
        p[i].y = laserCloudIn.points[i].y;
        p[i].z = laserCloudIn.points[i].z;
        p[i].clusterID = -1;
        points_.push_back(p[i]);
        ++i;
    }
    std::cout << "num of converted point : " << points_.size() << std::endl;
}

int DBSCAN::expandCluster(Point point, int clusterID){
    std::vector<int> clusterSeeds = calculateCluster(point);

    if(clusterSeeds.size() < minPoints_){
        point.clusterID = -2; // -2 = NOISE
        return -3; // -3 = FAILURE
    }
    else{
        int index = 0;
        int indexCorePoint = 0;
        std::vector<int>::iterator iterSeeds;
        for(iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds){
            points_.at(*iterSeeds).clusterID = clusterID;
            if(points_.at(*iterSeeds).x == point.x && points_.at(*iterSeeds).y == point.y && points_.at(*iterSeeds).z == point.z){
                indexCorePoint = index;
            }
            ++index;
        }
        clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);

        for(std::vector<int>::size_type i =0, n = clusterSeeds.size(); i < n; ++i){
            std::vector<int> clusterNeighbors = calculateCluster(points_.at(clusterSeeds[i]));
            if(clusterNeighbors.size() >= minPoints_){
                std::vector<int>::iterator iterNeighbors;
                for(iterNeighbors = clusterNeighbors.begin(); iterNeighbors != clusterNeighbors.end(); ++iterNeighbors){
                    if(points_.at(*iterNeighbors).clusterID == -1 || points_.at(*iterNeighbors).clusterID == -2){ // -1 = UNCLASIFIED, -2 = NOISE
                        clusterSeeds.push_back(*iterNeighbors);
                        n = clusterSeeds.size();
                    }
                    points_.at(*iterNeighbors).clusterID = clusterID;
                }
            }
        }
        return 0; // 0 = SUCCESS
    }
}

std::vector<int> DBSCAN::calculateCluster(Point point){
    int index = 0;
    std::vector<Point>::iterator iter;
    std::vector<int> clusterIndex;
    for(iter = points_.begin(); iter != points_.end(); ++iter){
        if(calculateDistance(point, *iter) <= epsilon_){
            clusterIndex.push_back(index);
        }
        index++;
    }
    return clusterIndex;
}

double DBSCAN::calculateDistance(const Point& pointCore, const Point& pointTarget){
    return pow(pointCore.x - pointTarget.x,2) + pow(pointCore.y - pointTarget.y, 2) + pow(pointCore.z - pointTarget.z,2);
}

void DBSCAN::print_result(){
    int num_points = points_.size();
    int i = 0;
    std::cout << "Number of points : " << num_points << std::endl;
    while(i < num_points){
        std::cout << points_[i].x << ", " << points_[i].y << ", " << points_[i].z << std::endl;
        ++i;
    }

}

void DBSCAN::dbscan_callback(const sensor_msgs::PointCloud2ConstPtr &input_non_ground_msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_non_ground_msg, *laserCloudIn);

    convertPoint(*laserCloudIn);

    int clusterID = 1;
    std::vector<Point>::iterator iter;
    for(iter = points_.begin(); iter != points_.end(); ++iter){
        if(iter->clusterID == -1){
            if(expandCluster(*iter, clusterID) != -3){ // -3 = FAILURE
                clusterID += 1;
            }
        }
    }
    std::cout << "num of cluster : " << clusterID << std::endl;
    // print_result();

    points_.clear();
}


int main(int argc, char** argv){
    ros::init(argc, argv, "dbscan");
    DBSCAN node;
    ros::spin();
    return 0;
}
