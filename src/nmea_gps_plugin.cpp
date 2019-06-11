#include <nmea_gps_plugin/nmea_gps_plugin.h>

namespace gazebo
{
    NmeaGpsPlugin::NmeaGpsPlugin()
    {

    }

    NmeaGpsPlugin::~NmeaGpsPlugin()
    {

    }

    void NmeaGpsPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        world_ptr_ = model->GetWorld();
        // load parameters
        if (!sdf->HasElement("robotNamespace"))
        {
            namespace_.clear();
        }
        else
        {
            namespace_ = sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
        }
        if (!sdf->HasElement("bodyName"))
        {
            link_ptr_ = model->GetLink();
            link_name_ = link_ptr_->GetName();
        }
        else
        {
            link_name_ = sdf->GetElement("bodyName")->GetValue()->GetAsString();
            link_ptr_ = model_ptr_->GetLink(link_name_);
        }
        reference_longitude_ = nmea_gps_plugin::default_param::reference_longitude;
        reference_latitude_ = nmea_gps_plugin::default_param::reference_latitude;
        reference_altitude_ = nmea_gps_plugin::default_param::reference_altitude;
        reference_heading_ = nmea_gps_plugin::default_param::reference_heading;
        nmea_topic_ = nmea_gps_plugin::default_param::nmea_topic;
        publish_rate_ = nmea_gps_plugin::default_param::publish_rate;
        if (sdf->HasElement("frameId"))
        {
            frame_id_ = sdf->GetElement("frameId")->GetValue()->GetAsString();
        }
        if (sdf->HasElement("topicName"))
        {
            nmea_topic_ = sdf->GetElement("topicName")->GetValue()->GetAsString();
        }
        if (sdf->HasElement("publishRate"))
        {
            sdf->GetElement("publishRate")->GetValue()->Get(publish_rate_);
        }
        if (sdf->HasElement("referenceLatitude"))
        {
            sdf->GetElement("referenceLatitude")->GetValue()->Get(reference_latitude_);
        }
        if (sdf->HasElement("referenceLongitude"))
        {
            sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
        }
        if (sdf->HasElement("referenceHeading"))
        {
            if (sdf->GetElement("referenceHeading")->GetValue()->Get(reference_heading_))
            {
                reference_heading_ = reference_heading_*M_PI/180.0;
            }
        }
        if (sdf->HasElement("referenceAltitude"))
        {
            sdf->GetElement("referenceAltitude")->GetValue()->Get(reference_altitude_);
        }
        node_handle_ = ros::NodeHandle(namespace_);
        nmea_pub_ = node_handle_.advertise<nmea_msgs::Sentence>(nmea_topic_,1);
        initial_pose_.position.longitude = reference_longitude_;
        initial_pose_.position.latitude = reference_latitude_;
        initial_pose_.position.altitude = reference_altitude_;
        geometry_msgs::Vector3 vec;
        vec.x = 0.0;
        vec.y = 0.0;
        vec.z = reference_heading_;
        initial_pose_.orientation = quaternion_operation::convertEulerAngleToQuaternion(vec);
        initial_utim_pose_ = geodesy::UTMPose(initial_pose_);
        update_timer_.setUpdateRate(publish_rate_);
        update_timer_.Load(world_ptr_, sdf);
        update_connection_ = update_timer_.Connect(boost::bind(&NmeaGpsPlugin::Update, this));
        return;
    }

    nmea_gps_plugin::byte getCheckSum(std::string sentence)
    {
        nmea_gps_plugin::byte ret;
        for(int i=0; i<sentence.size(); i++)
        {
            ret ^= (nmea_gps_plugin::byte)sentence[i];
        }
        return ret;
    }

    void NmeaGpsPlugin::Reset()
    {
        update_timer_.Reset();
        return;
    }

    void NmeaGpsPlugin::Update()
    {
        common::Time sim_time = world_ptr_->SimTime();
        double dt = update_timer_.getTimeSinceLastUpdate().Double();
        ignition::math::Pose3d pose = link_ptr_->WorldPose();
        ignition::math::Vector3d velocity = link_ptr_->WorldLinearVel();
        return;
    }

    GZ_REGISTER_MODEL_PLUGIN(NmeaGpsPlugin)
}