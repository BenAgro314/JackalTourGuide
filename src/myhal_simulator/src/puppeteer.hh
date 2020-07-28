#ifndef PUPPETEER_HH
#define PUPPETEER_HH

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <vector>
#include <map>
#include <utility>
#include "quadtree.hh"
#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include "vehicles.hh"
#include <std_srvs/Empty.h>
#include "frame.hh"
#include "gazebo/msgs/msgs.hh"
#include <geometry_msgs/PoseStamped.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef boost::shared_ptr<SmartCam> SmartCamPtr;

class Puppeteer: public gazebo::WorldPlugin{

    private:

    
        gazebo::event::ConnectionPtr update_connection;

        gazebo::physics::WorldPtr world;

        sdf::ElementPtr sdf;

        std::vector<boost::shared_ptr<Vehicle>> vehicles;

        std::vector<gazebo::physics::EntityPtr> collision_entities;

        std::string building_name;

        std::string robot_name = "";

        gazebo::physics::ModelPtr robot = nullptr;

        SmartCamPtr smart_cam = nullptr;

        gazebo::physics::EntityPtr building; 

        double update_freq = 60;

        gazebo::common::Time last_update;

        std::map<std::string, double> vehicle_params;

        std::map<std::string, double> boid_params;
        
        boost::shared_ptr<QuadTree> static_quadtree; 

        boost::shared_ptr<QuadTree> vehicle_quadtree; 

        boost::shared_ptr<QuadTree> vehicle_quadtree2; 

        ignition::math::Box building_box; 

        std::vector<boost::shared_ptr<Follower>> follower_queue;

        std::vector<gazebo::physics::LinkPtr> robot_links;

        ignition::math::Pose3d sensor_pose;

        std::string user_name, start_time, tour_name;

        boost::shared_ptr<Costmap> costmap;

        ros::NodeHandle nh;

        std::vector<std::vector<ignition::math::Vector3d>> paths;

        ros::Publisher path_pub;

        
    public: 
        
        void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);

        void OnUpdate(const gazebo::common::UpdateInfo &_info);

        void ReadSDF();

        void ReadParams();

        boost::shared_ptr<Vehicle> CreateVehicle(gazebo::physics::ActorPtr actor);

};


#endif
