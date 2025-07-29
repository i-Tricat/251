#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

// 발행할 costmap
ros::Publisher costmap_pub;

const float map_resolution = 0.1;  // Costmap 해상도 (셀당 크기, 0.1m)
const int map_width = 200;         // 맵의 너비 (셀 단위)
const int map_height = 200;        // 맵의 높이 (셀 단위)
const float map_origin_x = -10.0;  // 맵의 원점 X 좌표 (맵 중심이 0, 0이 되도록 설정)
const float map_origin_y = -10.0;  // 맵의 원점 Y 좌표

// 포인트 클라우드 콜백 함수
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // 포인트 클라우드 데이터를 PCL 포맷으로 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    // OccupancyGrid 메시지 생성
    nav_msgs::OccupancyGrid costmap;
    costmap.header.frame_id = "map";
    costmap.header.stamp = ros::Time::now();

    costmap.info.resolution = map_resolution;     // 해상도 설정
    costmap.info.width = map_width;               // 그리드 너비 설정
    costmap.info.height = map_height;             // 그리드 높이 설정
    costmap.info.origin.position.x = map_origin_x;  // 맵 원점 설정 (왼쪽 하단)
    costmap.info.origin.position.y = map_origin_y;
    costmap.info.origin.position.z = 0.0;

    // 그리드 데이터 초기화 (-1: 미탐색, 0: free, 100: 장애물)
    costmap.data.resize(map_width * map_height, -1);

    // 포인트 클라우드에서 XY 좌표를 그리드 셀로 변환
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;

        // 포인트가 맵 범위 내에 있는지 확인
        int grid_x = static_cast<int>((x - map_origin_x) / map_resolution);
        int grid_y = static_cast<int>((y - map_origin_y) / map_resolution);

        if (grid_x >= 0 && grid_x < map_width && grid_y >= 0 && grid_y < map_height)
        {
            // 2D 그리드를 1D 배열로 변환하여 인덱스 계산
            int index = grid_y * map_width + grid_x;

            // 해당 셀을 장애물로 표시 (여기서는 간단하게 100으로 설정)
            costmap.data[index] = 100;
        }
    }

    // Costmap 발행
    costmap_pub.publish(costmap);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "costmap_node");
    ros::NodeHandle nh;

    // 포인트 클라우드를 구독하고 costmap을 발행
    ros::Subscriber sub = nh.subscribe("/output", 1, cloudCallback);
    costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("costmap", 1);

    ros::spin();
}
