
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
  public: double obstacle_margin{0.2};

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

	  if (_sdf->HasElement("obstacle_margin")){

      	this->dataPtr->obstacle_margin = _sdf->GetElement("obstacle_margin")->Get<double>();

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

		//find maximum and min y value of building 
	double min_y = 10000;
	double max_y = -10000;
	double min_x = 10000;
	double max_x = -10000;

	for (auto const& link_list: this->dataPtr->building_links){
			
		for (auto link: link_list.second){ //iterate over all links of all buildings 
			ignition::math::Box box = link->BoundingBox();
			ignition::math::Vector3d min_corner = box.Min();
			ignition::math::Vector3d max_corner = box.Max();
			max_y = std::max(max_y, std::max(min_corner.Y(), max_corner.Y()));
			min_y = std::min(min_y, std::min(min_corner.Y(), max_corner.Y()));
			max_x = std::max(max_x, std::max(min_corner.X(), max_corner.X()));
			min_x = std::min(min_x, std::min(min_corner.X(), max_corner.X()));			}
	}


	std::printf("min_y %f, max_y %f, min_x %f, max_x %f\n", min_y, max_y, min_x, max_x);
	

	bool found_valid_place = false;
	ignition::math::Vector3d res_pos;
	while (!found_valid_place){
		


		double h = ignition::math::Rand::DblUniform(min_y, max_y);
		ignition::math::Line3d h_line = ignition::math::Line3d(-10000, h, 10000,h);

		std::vector<ignition::math::Vector3d> intersections;

		for (auto const& link_list: this->dataPtr->building_links){
			
			for (auto link: link_list.second){ //iterate over all links of all buildings 
				std::vector<ignition::math::Line3d> edges = this->get_edges(link);

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
				
			std::vector<ignition::math::Line3d> edges = this->get_edges(model);

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

			if (seg.Length() > 2*this->dataPtr->obstacle_margin){ //is the segment a valid placement point
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

		double length = ignition::math::Rand::DblUniform(this->dataPtr->obstacle_margin, seg.Length() - this->dataPtr->obstacle_margin);

		res_pos = ignition::math::Vector3d(1,0,0)*length + seg[0];

		//ensure that we are safe:
		/*
		1. iterate over all bounding boxes:
		2. check if our position is too close (normally) to any of the bounding boxes edges
		3. check if we are in a bounding box 
		4. if no to 2. and 3. , the positon is safe 
		*/

		for (auto const& link_list: this->dataPtr->building_links){
			
			for (auto link: link_list.second){ //iterate over all links of all buildings 
				auto normal = this->min_normal(res_pos, link);
				if (normal.Length() < this->dataPtr->obstacle_margin || this->point_in_object(res_pos, link)){
					found_valid_place = false;
					
				}
			}
		}

		if (res_pos.X() <= min_x+this->dataPtr->obstacle_margin || res_pos.X() >= max_x-this->dataPtr->obstacle_margin){
			found_valid_place = false;
			
		}
		
		
	}

	auto pose = ignition::math::Pose3d(res_pos.X(), res_pos.Y(), this->dataPtr->z_pos, 0, 0, 0);
	std::printf("(%f, %f, %f)\n", res_pos.X(), res_pos.Y(), res_pos.Z());
	this->dataPtr->self->SetWorldPose(pose , true, true);
  
}

/*
helper function that takes in an entity and returns a list of lines of its bounding box with Z=0
*/

std::vector<ignition::math::Line3d> RandomPlacement::get_edges(gazebo::physics::EntityPtr entity){
	ignition::math::Box box = entity->BoundingBox();
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

	return edges;
}

std::vector<ignition::math::Vector3d> RandomPlacement::get_corners(gazebo::physics::EntityPtr entity){
	ignition::math::Box box = entity->BoundingBox();
	ignition::math::Vector3d min_corner = box.Min();
	ignition::math::Vector3d max_corner = box.Max();
	min_corner.Z() = 0;
	max_corner.Z() = 0;


	//TODO: ensure that these methods work using Line3d
	ignition::math::Vector3d bot_l = min_corner;
	ignition::math::Vector3d bot_r = ignition::math::Vector3d(max_corner.X(),min_corner.Y(),0);
	ignition::math::Vector3d top_l = ignition::math::Vector3d(min_corner.X(),max_corner.Y(),0);
	ignition::math::Vector3d top_r = max_corner;
				
	std::vector<ignition::math::Vector3d> corners = {bot_l, bot_r, top_l, top_r}; // store all edges of link bounding box

	return corners;
}


bool RandomPlacement::point_in_object(ignition::math::Vector3d pos,gazebo::physics::EntityPtr entity){
	ignition::math::Box box = entity->BoundingBox();
	ignition::math::Vector3d min_corner = box.Min();
	ignition::math::Vector3d max_corner = box.Max();
	min_corner.Z() = 0;
	max_corner.Z() = 0;

	if (pos.X() >= std::min(min_corner.X(), max_corner.X()) 
	&& pos.X() <= std::max(min_corner.X(), max_corner.X())
	&& pos.Y() >= std::min(min_corner.Y(), max_corner.Y())
	&& pos.Y() <= std::max(min_corner.Y(), max_corner.Y())){
		return  true;
	} else{
		return false;
	}
}


//returns the shortest normal vector between pos and one of the edges on the bounding box of entity
// will return the shortest corner distance if the normal does not exist 

ignition::math::Vector3d RandomPlacement::min_normal(ignition::math::Vector3d pos, gazebo::physics::EntityPtr entity){
	std::vector<ignition::math::Line3d> edges = this->get_edges(entity);

	ignition::math::Vector3d min_normal;
	double min_mag = 1000000;
	bool found = false;

	for (ignition::math::Line3d edge: edges){

					
		ignition::math::Vector3d edge_vector = edge.Direction(); // vector in direction of edge 
					
		ignition::math::Vector3d pos_vector = ignition::math::Vector3d(pos.X()-edge[0].X(), pos.Y()-edge[0].Y(), 0);// vector from edge corner to actor pos
					
		ignition::math::Vector3d proj = ((pos_vector.Dot(edge_vector))/(edge_vector.Dot(edge_vector)))*edge_vector; // project pos_vector onto edge_vector
			
		//check if the projected point is within the edge
		if (edge.Within(proj+edge[0])){
			//compute normal
			ignition::math::Vector3d normal = pos_vector-proj;
						
			if (normal.Length() < min_mag){
				min_normal = normal;
				min_mag = normal.Length();
				found = true;
			}

		}
				
	}

	if (!found){ // iterate over all corners and find the closest one 
		min_mag = 1000000;
		auto corners = this->get_corners(entity);
		for (auto corner: corners){
			if (pos.Distance(corner) < min_mag){
				min_mag = pos.Distance(corner);
				min_normal = pos-corner;
			}
		}
	}

	return min_normal;
}