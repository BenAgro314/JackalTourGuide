
#include <functional>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Line2.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Box.hh>
#include <vector>
#include <algorithm>
#include <string>
#include <gazebo/common/Animation.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <map>

#include "random_place_plugin.hh"

using namespace gazebo;
using namespace servicesim;

GZ_REGISTER_MODEL_PLUGIN(servicesim::RandomPlacement)

class servicesim::RandomPlacementPrivate
{

  /// \brief Pointer to self.
  public: physics::ModelPtr self{nullptr};

  /// \brief Margin by which to increase an obstacle's bounding box on every
  /// direction (2x per axis).
  public: double obstacleMargin{0.2};

  /// \brief List of models to avoid
  public: std::vector<std::string> buildings;

  public: std::map<std::string,gazebo::physics::Link_V> building_links;

  public: double z_pos = 1;

};

/////////////////////////////////////////////////
RandomPlacement::RandomPlacement()
    : dataPtr(new RandomPlacementPrivate)
{
}

/////////////////////////////////////////////////
void RandomPlacement::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  	this->dataPtr->self = _model;

 	 // Read in the obstacles
  	if (_sdf->HasElement("building"))
  	{
    	auto obstacleElem = _sdf->GetElement("building");
    	while (obstacleElem){
      		auto name = obstacleElem->Get<std::string>();
      		this->dataPtr->buildings.push_back(name);
      		obstacleElem = obstacleElem->GetNextElement("building");
    	}
  	}

	if (_sdf->HasElement("z_pos")){

      	this->dataPtr->z_pos = _sdf->GetElement("z_pos")->Get<double>();

  	}

	
	auto world = this->dataPtr->self->GetWorld();
   	for (unsigned int i = 0; i < world->ModelCount(); ++i) {
		// iterate over all models. add to building link map as nessecary
		
		auto model = world->ModelByIndex(i);
		

		if (std::find(this->dataPtr->buildings.begin(),this->dataPtr->buildings.end(), model->GetName()) != this->dataPtr->buildings.end()){ //add any desired building name
			
			this->dataPtr->building_links[model->GetName()] = model->GetLinks();
		}
   	}

	/*
	random world placement

	1. if not placed: choose a random free spot in the building to start 
	3. find the minimum and maximum y of the building 
	2. draw a horizontal line at a random y
	3. obtain a list of all intersections with the bouding boxes of objects, buildings, and people
	4. every two intersections will actually count as one intersection (because each bouding box will be intersected twice)
	5. construct a list of "free" line segements where the model could be placed 
	6. randomly choose a suitable line (one that is long enough), and place the model in that position
	7. if there are no suitable segments, rechoose horizontal line
	*/
	
	//TODO: fix this if statement

	bool found_valid_place = false;
	ignition::math::Vector3d res_pos;
	while (!found_valid_place){
		//find maximum and min y value of building 
		double min_y = 10000;
		double max_y = -10000;
		
		for (auto const& link_list: this->dataPtr->building_links){
			
			for (auto link: link_list.second){ //iterate over all links of all buildings 
				ignition::math::Box box = link->BoundingBox();
				ignition::math::Vector3d min_corner = box.Min();
				ignition::math::Vector3d max_corner = box.Max();

				max_y = std::max(max_y, std::max(min_corner.Y(), max_corner.Y()));
				min_y = std::min(min_y, std::min(min_corner.Y(), max_corner.Y()));
			}
		}

		//std::printf("min_y %f, max_y %f\n", min_y, max_y);




		double h = ignition::math::Rand::DblUniform(min_y, max_y);
		ignition::math::Line3d h_line = ignition::math::Line3d(-10000, h, 10000,h);

		std::vector<ignition::math::Vector3d> intersections;

		for (auto const& link_list: this->dataPtr->building_links){
			
			for (auto link: link_list.second){ //iterate over all links of all buildings 
				ignition::math::Box box = link->BoundingBox();
				ignition::math::Vector3d min_corner = box.Min();
				ignition::math::Vector3d max_corner = box.Max();
				min_corner.Z() = 0;
				max_corner.Z() = 0;


				//TODO: ensure that these methods work using Line3d
				ignition::math::Line3d left = ignition::math::Line3d(min_corner.X(),min_corner.Y(),min_corner.X(), max_corner.Y());
				ignition::math::Line3d right = ignition::math::Line3d(max_corner.X(),min_corner.Y(),max_corner.X(), max_corner.Y());
				ignition::math::Line3d top = ignition::math::Line3d(min_corner.X(),max_corner.Y(),max_corner.X(), max_corner.Y());
				ignition::math::Line3d bot = ignition::math::Line3d(min_corner.X(),min_corner.Y(),max_corner.X(), min_corner.Y());
				
				std::vector<ignition::math::Line3d> edges = {left, right, top, bot}; // store all edges of link bounding box

				for (ignition::math::Line3d edge: edges){
					ignition::math::Vector3d test_intersection;
					if (edge.Intersect(h_line,test_intersection)){
						intersections.push_back(test_intersection);
					}
				}
			}
		}

		//check intersections with models 
		for (unsigned int i = 0; i < world->ModelCount(); ++i) {
			auto model = world->ModelByIndex(i);
			if (std::find(this->dataPtr->buildings.begin(),this->dataPtr->buildings.end(), model->GetName()) != this->dataPtr->buildings.end()){ //already dealt with buildings
				continue;
			}
			if ((model == this->dataPtr->self) || (model->GetName() == "ground_plane")) { //ignore self and ground 
				continue; 
			}

			ignition::math::Box box = model->BoundingBox();
			ignition::math::Vector3d min_corner = box.Min();
			ignition::math::Vector3d max_corner = box.Max();
			min_corner.Z() = 0;
			max_corner.Z() = 0;


			//TODO: ensure that these methods work using Line3d
			ignition::math::Line3d left = ignition::math::Line3d(min_corner.X(),min_corner.Y(),min_corner.X(), max_corner.Y());
			ignition::math::Line3d right = ignition::math::Line3d(max_corner.X(),min_corner.Y(),max_corner.X(), max_corner.Y());
			ignition::math::Line3d top = ignition::math::Line3d(min_corner.X(),max_corner.Y(),max_corner.X(), max_corner.Y());
			ignition::math::Line3d bot = ignition::math::Line3d(min_corner.X(),min_corner.Y(),max_corner.X(), min_corner.Y());
				
			std::vector<ignition::math::Line3d> edges = {left, right, top, bot}; // store all edges of link bounding box

			for (ignition::math::Line3d edge: edges){
				ignition::math::Vector3d test_intersection;
				if (edge.Intersect(h_line,test_intersection)){
					intersections.push_back(test_intersection);
				}
			}


		}

		//now intersections should be filled 

		//std::cout << intersections.size() << std::endl;

		std::vector<ignition::math::Line3d> segments;

		for (int i =1; i<intersections.size()-1; i+=2){
			ignition::math::Vector3d start = intersections[i];
			ignition::math::Vector3d end = intersections[i+1];

			ignition::math::Line3d seg = ignition::math::Line3d(start, end);

			if (seg.Length() > 2*this->dataPtr->obstacleMargin){ //is the segment a valid placement point
				segments.push_back(seg);
			}
		}

		if (segments.size() <=0){
			continue;
		}

		found_valid_place = true;

		int index = ignition::math::Rand::IntUniform(0,segments.size()-1);
		
		auto seg = segments[index];

		//we must ensure that the person is placed at a valid point on the segement (not too close to either side)

		double length = ignition::math::Rand::DblUniform(this->dataPtr->obstacleMargin, seg.Length() - this->dataPtr->obstacleMargin);

		res_pos = ignition::math::Vector3d(1,0,0)*length + seg[0];
		
	}

	auto pose = ignition::math::Pose3d(res_pos.X(), res_pos.Y(), this->dataPtr->z_pos, 0, 0, 0);
	std::printf("(%f, %f, %f)\n", res_pos.X(), res_pos.Y(), res_pos.Z());
	this->dataPtr->self->SetWorldPose(pose , false, false);
  
}


