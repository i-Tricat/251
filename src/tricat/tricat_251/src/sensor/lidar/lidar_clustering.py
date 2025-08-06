#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from math import pow, sqrt
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray
from tricat_msgs.msg import Obstacle, ObstacleList
from dynamic_reconfigure.server import Server
# from tricat_241.cfg import LidarControllerConfig
# import gps.gnss_converter as gc
import visual.show as sh
from sensor.lidar.lidar_calc import Point, PointSet

class LidarController:
    def __init__(self, onoff):
        self.onoff = onoff

        # Default parameters
        self.min_input_points_size = rospy.get_param("~min_input_points_size", 1)
        self.max_gap_in_set = rospy.get_param("~max_gap_in_set", 1)
        self.point_set_size = rospy.get_param("~point_set_size", 1)
        self.max_dist_to_ps_line = rospy.get_param("~max_dist_to_ps_line", 3)
        self.min_wall_length = rospy.get_param("~min_wall_length", 50)
        self.wall_particle_length = rospy.get_param("~wall_particle_length", 50)

        # Initialize dynamic reconfigure server
        self.server = Server(LidarControllerConfig, self.reconfigure_callback)

    def reconfigure_callback(self, config, level):
        """Callback function that updates parameters dynamically"""
        self.min_input_points_size = config.min_input_points_size
        self.max_gap_in_set = config.max_gap_in_set
        self.point_set_size = config.point_set_size
        self.max_dist_to_ps_line = config.max_dist_to_ps_line
        self.min_wall_length = config.min_wall_length
        self.wall_particle_length = config.wall_particle_length

        rospy.loginfo("Reconfigure Request: min_input_points_size = {min_input_points_size}, max_gap_in_set = {max_gap_in_set}, point_set_size = {point_set_size}, max_dist_to_ps_line = {max_dist_to_ps_line}, min_wall_length = {min_wall_length}, wall_particle_length = {wall_particle_length}".format(**config))
        return config

    def param_pub(self):
        return self.min_input_points_size, self.max_gap_in_set, self.point_set_size, self.max_dist_to_ps_line, self.min_wall_length, self.wall_particle_length

class LidarConverter:
    def __init__(self, param):
        self.boat_x, self.boat_y = 0, 0  # current boat position
        self.goal_x, self.goal_y = 1000, 1000

        # Params
        self.min_input_points_size, self.max_gap_in_set, self.point_set_size, self.max_dist_to_ps_line, self.min_wall_length, self.wall_particle_length = param

        # Scanning / converted data
        self.input_points = []
        self.point_sets_list = []
        self.buoy_particle = []
        self.obstacles = []

        # Sub, pub
        rospy.Subscriber("/output", PointCloud2, self.lidar_raw_callback, queue_size=1)
        # rospy.Subscriber("/scan", LaserScan, self.lidar_raw_callback, queue_size=1)
        self.obstacle_pub = rospy.Publisher("/obstacles", ObstacleList, queue_size=10)

    def get_param(self, param):
        self.min_input_points_size, self.max_gap_in_set, self.point_set_size, self.max_dist_to_ps_line, self.min_wall_length, self.wall_particle_length = param

    # def lidar_raw_callback(self, msg):
    #     """Subscribe to lidar scanning data and convert to cartesian coordinates"""
    #     self.input_points = []
    #     self.point_sets_list = []
    #     self.obstacles = []

    #     phi = msg.angle_min
    #     for r in msg.ranges:
    #         if msg.range_min <= r <= msg.range_max:
    #             p = Point.polar_to_cartesian(r, phi)
    #             self.input_points.append(p)
    #         phi += msg.angle_increment

    def lidar_raw_callback(self, msg):
        """Subscribe lidar scanning data
        Notes:
            * convert PointCloud2 data to (x, y) points
        """
        # initialize all data lists
        self.input_points = []
        self.point_sets_list = []
        self.obstacles = []

        # Convert PointCloud2 data to (x, y) points
        for point in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            p = Point(x=point[0], y=point[1])
            self.input_points.append(p)

    def process_points(self):
        """Clustering process"""
        if len(self.input_points) < self.min_input_points_size:
            return 

        self.group_points()
        for ps in self.point_sets_list:
            self.split_group(ps)
        self.classify_groups()

    def group_points(self):
        """Group adjacent raw scanning points"""
        point_set = PointSet()
        point_set.append_point(self.input_points[0])
        for p in self.input_points:
            if p.dist_btw_points(point_set.end) > self.max_gap_in_set:
                if point_set.set_size > self.point_set_size:
                    self.point_sets_list.append(point_set)

                del point_set
                point_set = PointSet()
                point_set.append_point(p)
            else:
                point_set.append_point(p)
        self.point_sets_list.append(point_set)

        # Remove the first duplicate point
        self.point_sets_list[0].begin = self.point_sets_list[0].point_set[1]
        self.point_sets_list[0].set_size -= 1
        del self.point_sets_list[0].point_set[0]

    def split_group(self, ps):
        """Split group into smaller ones"""
        max_distance = 0
        split_idx = 0
        point_idx = 0

        for p in ps.point_set:
            dist_to_ps_line = ps.dist_to_point(p)
            if dist_to_ps_line > max_distance:
                max_distance = dist_to_ps_line
                split_idx = point_idx
            point_idx += 1

        if max_distance > self.max_dist_to_ps_line:
            if split_idx < self.point_set_size or (ps.set_size - split_idx) < self.point_set_size:
                return

            ps1 = PointSet()
            ps1.input_point_set(ps.point_set[:split_idx])
            ps2 = PointSet()
            ps2.input_point_set(ps.point_set[split_idx:])

            if ps in self.point_sets_list:
                index = self.point_sets_list.index(ps)
                self.point_sets_list.insert(index, ps1)
                self.point_sets_list.insert(index + 1, ps2)
                del self.point_sets_list[index]

                self.split_group(ps1)
                self.split_group(ps2)
            else:
                pass
                #rospy.logwarn(f"PointSet {ps} is not in the list")


    def classify_groups(self):
        """Classify groups as Walls or Buoys"""
        for ps in self.point_sets_list:
            if ps.dist_begin_to_end() > self.min_wall_length:
                self.split_wall(ps)
            else:
                self.second_rearrange_buoy(ps)

    def split_wall(self, ps):
        """Split long walls into smaller segments"""
        wall_particle = PointSet()
        wall_particle.append_point(ps.begin)
        for p in ps.point_set:
            if p.dist_btw_points(wall_particle.begin) > self.wall_particle_length:
                self.obstacles.append(wall_particle)
                del wall_particle
                wall_particle = PointSet()
            wall_particle.append_point(p)
        self.obstacles.append(wall_particle)

    def second_rearrange_buoy(self, ps):
        """Rearrange buoy particles"""
        buoy_particle = PointSet()
        min_dist = float('inf')

        for p in ps.point_set:
            buoy_particle.append_point(p)
            res = sqrt(pow(p.y - self.boat_y, 2.0) + pow(p.x - self.boat_x, 2.0))
            if res < min_dist:
                min_dist = res
                closed_point = [p.x, p.y]

        line_inclination = (ps.point_set[-1].y - ps.point_set[0].y) / (ps.point_set[-1].x - ps.point_set[0].x + 0.00000000000000001)
        new_line_y = -1 * line_inclination * closed_point[0] + closed_point[1]

        if line_inclination == 0:
            new_begin_point_x = ps.point_set[0].x
            new_begin_point_y = line_inclination * ps.point_set[0].x + new_line_y
            new_end_point_x = ps.point_set[-1].x
            new_end_point_y = line_inclination * ps.point_set[-1].x + new_line_y
        else:
            conversion_line_inclination = 1 / -line_inclination
            begin_conversion_line_inclination_y = -conversion_line_inclination * ps.point_set[0].x + ps.point_set[0].y
            end_conversion_line_inclination_y = -conversion_line_inclination * ps.point_set[-1].x + ps.point_set[-1].y
            new_begin_point_x = ((begin_conversion_line_inclination_y - new_line_y) / (line_inclination - conversion_line_inclination))
            new_begin_point_y = (line_inclination * (begin_conversion_line_inclination_y - new_line_y) / (line_inclination - conversion_line_inclination) + new_line_y)
            new_end_point_x = ((end_conversion_line_inclination_y - new_line_y) / (line_inclination - conversion_line_inclination))
            new_end_point_y = (line_inclination * (end_conversion_line_inclination_y - new_line_y) / (line_inclination - conversion_line_inclination) + new_line_y)

        buoy_particle.begin.x = new_begin_point_x
        buoy_particle.begin.y = new_begin_point_y
        buoy_particle.end.x = new_end_point_x
        buoy_particle.end.y = new_end_point_y

        self.obstacles.append(buoy_particle)

    def publish_obstacles(self):
        """Publish obstacles in (x, y) coordinate format"""
        ob_list = ObstacleList()
        for ob in self.obstacles:
            obstacle = Obstacle()
            obstacle.begin.x = ob.begin.x
            obstacle.begin.y = ob.begin.y
            obstacle.end.x = ob.end.x
            obstacle.end.y = ob.end.y
            ob_list.obstacle.append(obstacle)
        self.obstacle_pub.publish(ob_list)

class LidarViewer:
    def __init__(self):
        self.obstacles = []
        rospy.Subscriber("/obstacles", ObstacleList, self.obstacles_callback, queue_size=10)
        self.rviz_pub = rospy.Publisher("/rviz_visual", MarkerArray, queue_size=10)

    def obstacles_callback(self, msg):
        self.obstacles = msg.obstacle

    def publish_rviz(self):
        ids = list(range(0, 100))
        obstacle = []
        for ob in self.obstacles:
            obstacle.append([ob.begin.x, ob.begin.y])
            obstacle.append([ob.end.x, ob.end.y])
        obstacle = sh.linelist_rviz(
            name="obstacle", id=ids.pop(), lines=obstacle, color_r=10, color_g=81, color_b=204, scale=0.1)
        all_markers = sh.marker_array_rviz([obstacle])
        self.rviz_pub.publish(all_markers)

def main():
    rospy.init_node("LidarConverter", anonymous=False)

    onoff = rospy.get_param("controller", True)
    lidar_control = LidarController(onoff)
    param = lidar_control.param_pub()
    lidar_convert = LidarConverter(param)
    visualizer = LidarViewer()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        param = lidar_control.param_pub()
        lidar_convert.get_param(param)
        lidar_convert.process_points()
        lidar_convert.publish_obstacles()
        visualizer.publish_rviz()
        rate.sleep()

if __name__ == "__main__":
    main()
