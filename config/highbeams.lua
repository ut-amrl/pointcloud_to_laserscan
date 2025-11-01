pointcloud_to_laser = {
  range_min = 0.3;
  range_max = 70.0;
  angle_min = -math.pi;
  angle_max = math.pi;
  height_min = 0.5;
  height_max = 2.0;
  num_ranges = 1440;  -- angular resolution for laserscan (angle_max - angle_min)/num_ranges = 0.25deg here
  laser_topic = "velodyne_2dscan_highbeams";
  pointcloud_topic = "velodyne_points";
  debug_pointcloud_topic = "debug_pointcloud_highbeams";
}
