#ifndef NMEA_GPS_PLUGIN_NMEA_GPS_PLUGIN_H_INCLUDED
#define NMEA_GPS_PLUGIN_NMEA_GPS_PLUGIN_H_INCLUDED

// Headers in Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>

// Headers in ROS
#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <geographic_msgs/GeoPose.h>
#include <quaternion_operation/quaternion_operation.h>
#include <geodesy/utm.h>
#include <hector_gazebo_plugins/update_timer.h>

namespace nmea_gps_plugin
{
    namespace default_param
    {
        constexpr double reference_longitude = 0.0;
        constexpr double reference_latitude = 0.0;
        constexpr double reference_heading = 0.0;
        constexpr double reference_altitude = 0.0;
        constexpr double publish_rate = 1.0;
        const std::string nmea_topic = "/nmea/sentence";
    }

    typedef unsigned char byte;
}

namespace gazebo
{
    class NmeaGpsPlugin : public ModelPlugin
    {
        public:
            NmeaGpsPlugin();
            virtual ~NmeaGpsPlugin();
        protected:
            virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
            virtual void Reset();
            virtual void Update();
        private:
            physics::WorldPtr world_ptr_;
            physics::LinkPtr link_ptr_;
            physics::ModelPtr model_ptr_;
            ros::NodeHandle node_handle_;
            std::string namespace_;
            std::string link_name_;
            std::string frame_id_;
            std::string nmea_topic_;
            double reference_altitude_;
            double reference_longitude_;
            double reference_latitude_;
            double reference_heading_;
            double publish_rate_;
            ros::Publisher nmea_pub_;
            geographic_msgs::GeoPose initial_pose_;
            geodesy::UTMPose initial_utim_pose_;
            UpdateTimer update_timer_;
            event::ConnectionPtr update_connection_;
            nmea_gps_plugin::byte getCheckSum(std::string sentence);
    };
}

#endif  //NMEA_GPS_PLUGIN_NMEA_GPS_PLUGIN_H_INCLUDED