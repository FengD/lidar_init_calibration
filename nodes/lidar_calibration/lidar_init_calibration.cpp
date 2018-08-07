#include <string>
#include <string.h>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <math.h>
#include <vector>
#include "yaml_editor.h"

// The accept topic of the point cloud
// And the file path of the generated yaml file
static std::string points_cloud_topic, output_file_path;
// The correction results of z, roll, pitch, yaw
static double roll_correction = 0, pitch_correction = 0, yaw_correction = 0, z_correction = 0;
// The counter which used to calculate the avg of the calibration result
static int counter = 0;
// If the process is finished
static bool isCalibrationFinished = false;
// The object which used to read or write the yaml file
static CalibrationYaml cYaml;
// The ground publisher
ros::Publisher ground_points_pub;
// the callback of the points cloud
static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input) {
  // check if the calibration is finished
  if(!isCalibrationFinished) {
    counter++;
    pcl::PointCloud<pcl::PointXYZI> points, ground_points, cloud_filtered;

    pcl::fromROSMsg(*input, points);
    // create an PassThrough object to filter the point cloud by the range of x, y
    pcl::PassThrough<pcl::PointXYZI> pass;
    // set false means keep the points in the range
    pass.setFilterLimitsNegative(false);
    // set the input cloud
    pass.setInputCloud(points.makeShared());
    // set the filed name used to finish the filter
    pass.setFilterFieldName("x");
    //  set the range
    pass.setFilterLimits(-15.0, 15.0);
    // get the result and save them in a point cloud
    pass.filter(points);
    // do the same thing for axe y
    pass.setInputCloud(points.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10.0, 2.0);
    pass.filter(cloud_filtered);
    // the coefficient of the plane function
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // the indices of the points in the ground
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    //use the SAC_RANSAC to make the segmentation
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    //set if set the setOptimizeCoefficients
    seg.setOptimizeCoefficients (true);
    //if the step before is true the following config is obligatory
    //set the segmentation model
    seg.setModelType (pcl::SACMODEL_PLANE);
    //set the segmentation method
    seg.setMethodType (pcl::SAC_RANSAC);
    // set the threshold of the distance
    seg.setDistanceThreshold (0.1);
    // set the input cloud
    seg.setInputCloud (cloud_filtered.makeShared());
    //realise the segmentation and save the indices of the ground points and the coefficients of the ground plane
    seg.segment (*inliers, *coefficients);
    // publish the ground (optional)
    sensor_msgs::PointCloud2 ground_points_msg;
    pcl::copyPointCloud(cloud_filtered, inliers->indices, ground_points);
    pcl::toROSMsg(ground_points, ground_points_msg);
    ground_points_msg.header = input->header;
    ground_points_pub.publish(ground_points_msg);
    // get the coefficient a, b, c, d: ax + by + cz + d = 0;
    coefficients->values.resize(4);
    double a = coefficients->values[0];
    double b = coefficients->values[1];
    double c = coefficients->values[2];
    double d = coefficients->values[3];
    // std::cout << coefficients->values[0] << std::endl;
    // std::cout << coefficients->values[1] << std::endl;
    // std::cout << coefficients->values[2] << std::endl;
    // std::cout << coefficients->values[3] << std::endl;

    // calculate the current point cloud correction value
    double coff = sqrt(a * a + b * b + c * c);
    // correction_yaw
    double yy = asin(c / coff) * 180 / M_PI;
    // correction_pitch
    double pp = asin(a / coff) * 180 / M_PI;
    // correction_roll
    double rr = asin(b / coff) * 180 / M_PI;
    // accumulate of the value
    roll_correction += rr;
    pitch_correction += pp;
    yaw_correction += yy;
    z_correction += d;

    std::cout << "roll_correction " << (roll_correction / counter) << std::endl;
    std::cout << "pitch_correction " << (pitch_correction / counter) << std::endl;
    std::cout << "yaw_correction " << (yaw_correction / counter) << std::endl;
    std::cout << "z_correction " << (z_correction / counter) << std::endl;

    // if the counter reach to 1000(can be changed), we calculate the avg
    if(counter == 1000) {
      isCalibrationFinished = true;
      Calibration_Lidar cLidar;
      cLidar.x = 0.0;
      cLidar.y = 0.0;
      cLidar.z = z_correction / counter;
      cLidar.roll = roll_correction / counter;
      cLidar.pitch = -pitch_correction / counter;
      // cLidar.yaw = 90 - yaw_correction / counter;
      cLidar.yaw = 0.0;
      // Write the results in an yaml file
      cYaml.yamlWrite(output_file_path, cLidar);
      std::cout << "The init calibration is finished, you could now stop the program!" << std::endl;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_calibration");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  if (private_nh.getParam("points_cloud_topic", points_cloud_topic) == false)
  {
    std::cout << "points_cloud_topic is not set." << std::endl;
    return 1;
  }

  if (private_nh.getParam("output_file_path", output_file_path) == false)
  {
    std::cout << "output_file_path is not set." << std::endl;
    return 1;
  }

  ros::Subscriber points_cloud_sub = nh.subscribe(points_cloud_topic, 1, points_callback);

  ground_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground", 10);

  ros::spin();
}
