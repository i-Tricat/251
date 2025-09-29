#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>  // laser → pointcloud 변환

class ScanToPointCloudBridge
{
public:
  ScanToPointCloudBridge()
  {
    scan_sub_ = nh_.subscribe("scan", 10, &ScanToPointCloudBridge::scanCallback, this);
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("bridge_pointcloud", 10);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    sensor_msgs::PointCloud2 pc;
    projector_.projectLaser(*scan, pc);
    pc.header = scan->header;
    pc_pub_.publish(pc);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher pc_pub_;
  laser_geometry::LaserProjection projector_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_to_pc_bridge");
  ScanToPointCloudBridge node;
  ros::spin();
  return 0;
}
