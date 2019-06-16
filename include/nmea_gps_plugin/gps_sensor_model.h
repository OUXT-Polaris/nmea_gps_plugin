#ifndef NMEA_GPS_PLUGIN_GPS_SENSOR_MODEL_H_INCLUDED
#define NMEA_GPS_PLUGIN_GPS_SENSOR_MODEL_H_INCLUDED

// Headers in ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geodesy/utm.h>
#include <quaternion_operation/quaternion_operation.h>

// Headers in STL
#include <random>

class GpsSensorModel
{
public:
    const double position_gaussian_noise;
    const double velocity_gaussian_noise;
    const double orientation_gaussian_noise;
    GpsSensorModel(double position_gaussian_noise,double orientation_gaussian_noise,double velocity_gaussian_noise);
    ~GpsSensorModel();
    geometry_msgs::Twist addGausiannNoise(geometry_msgs::Twist twist);
    geometry_msgs::Quaternion addGausiannNoise(geometry_msgs::Quaternion orientation);
    geodesy::UTMPoint addGausiannNoise(geodesy::UTMPoint point);
private:
    std::normal_distribution<> position_dist_;
    std::normal_distribution<> orientation_dist_;
    std::normal_distribution<> twist_dist_;
    std::random_device seed_gen_;
    std::default_random_engine engine_;
};
#endif  //NMEA_GPS_PLUGIN_GPS_SENSOR_MODEL_H_INCLUDED