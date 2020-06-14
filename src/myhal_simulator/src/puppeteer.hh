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

#define PUB false // publish catagorized point clouds?
#define PLY false // publish catagorized ply files?
#define RECORD true // record the model poses to a PLY file?
#define NAV true // publish a reduced pointcloud for navigation?

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

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

        gazebo::physics::EntityPtr building; 

        double update_freq = 60;

        gazebo::common::Time last_update;

        gazebo::common::Time last_pub;

        gazebo::common::Time last_retarget;

        double retarget_time = 10;

        std::map<std::string, double> vehicle_params;

        std::map<std::string, double> boid_params;
        
        boost::shared_ptr<QuadTree> static_quadtree; 

        boost::shared_ptr<QuadTree> vehicle_quadtree; 

        boost::shared_ptr<QuadTree> vehicle_quadtree2; 

        ignition::math::Box building_box; 

        std::vector<boost::shared_ptr<Follower>> follower_queue;

        std::vector<boost::shared_ptr<FlowField>> fields;

        std::vector<gazebo::physics::LinkPtr> robot_links;

        std::string user_name;

        std::string start_time;

        ignition::math::Pose3d sensor_pose;
        
        bool lidar_listener = false;

        ros::NodeHandle nh;

        ros::Subscriber sub;

        ros::Publisher ground_pub;

        ros::Publisher wall_pub;

        ros::Publisher moving_actor_pub;

        ros::Publisher still_actor_pub;

        ros::Publisher table_pub;

        ros::Publisher chair_pub;

        ros::Publisher nav_pub;

        ros::Publisher standing_actors;

        ros::ServiceClient pauseGazebo;

        ros::ServiceClient playGazebo;

        std_srvs::Empty emptySrv;

        bool publish_points = false;

        bool publish_ply = false;

        bool record_objects = false;

        bool publish_navigation = false;

        std::vector<BoxObject> ply_boxes;

        void Callback(const PointCloud::ConstPtr& msg);

        
    public: 
        
        void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);

        void OnUpdate(const gazebo::common::UpdateInfo &_info);

        void ReadSDF();

        void ReadParams();

        boost::shared_ptr<Vehicle> CreateVehicle(gazebo::physics::ActorPtr actor);

};


#endif