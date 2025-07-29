#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/search/kdtree.h>      
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;

// 콜백 함수: LiDAR 데이터를 받아서 처리
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // sensor_msgs/PointCloud2를 PCL 포맷으로 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    // 클러스터링 알고리즘 적용 등 데이터 처리
    // Voxel Grid 필터 적용
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f); // Leaf Size 설정 (X, Y, Z)
    vg.filter(*filtered_cloud);

    // KD 트리 기반 클러스터링
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(filtered_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);  // 포인트 간 최대 거리 (단위: 미터)
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_cloud);
    ec.extract(cluster_indices);

    // 정사영: 모든 포인트의 Z 좌표를 0으로 설정
    for (size_t i = 0; i < filtered_cloud->points.size(); ++i)
    {
        filtered_cloud->points[i].z = 0.0;
    }

    // 처리된 결과를 다시 sensor_msgs/PointCloud2로 변환하여 발행
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*filtered_cloud, output);  // 필터링 및 정사영된 클라우드를 메시지로 변환
    output.header.frame_id = input->header.frame_id;
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_cluster_node");
    ros::NodeHandle nh;

    // LiDAR 데이터를 구독하고 결과를 발행
    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloudCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    ros::spin();
}
