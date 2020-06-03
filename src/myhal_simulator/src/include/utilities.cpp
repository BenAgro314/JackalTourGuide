#include "utilities.hh"


std::vector<ignition::math::Line3d> utilities::get_edges(gazebo::physics::EntityPtr entity){
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

std::vector<ignition::math::Vector3d> utilities::get_corners(gazebo::physics::EntityPtr entity){
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

std::vector<ignition::math::Vector3d> utilities::get_box_corners(ignition::math::Box box){
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

//returns true if the projection of pos falls within the bounds of edge. If it does, it stores the normal vector between the edge and pos in normal (pointing from edge to point)

bool utilities::get_normal_to_edge(ignition::math::Vector3d pos, ignition::math::Line3d edge, ignition::math::Vector3d &normal){
    ignition::math::Vector3d edge_vector = edge.Direction(); 
					
	ignition::math::Vector3d pos_vector = ignition::math::Vector3d(pos.X()-edge[0].X(), pos.Y()-edge[0].Y(), 0);
	ignition::math::Vector3d proj = ((pos_vector.Dot(edge_vector))/(edge_vector.Dot(edge_vector)))*edge_vector; 

	if (edge.Within(proj+edge[0])){
		normal = pos_vector-proj;
        return true;
	} else{
        return false;
    }
}

bool utilities::inside_box(ignition::math::Box box, ignition::math::Vector3d point){
	ignition::math::Vector3d min_corner = box.Min();
	ignition::math::Vector3d max_corner = box.Max();

	
	return (point.X() < std::max(min_corner.X(), max_corner.X())
	&& point.X() > std::min(min_corner.X(), max_corner.X())
	&& point.Y() < std::max(min_corner.Y(), max_corner.Y())
	&& point.Y() > std::min(min_corner.Y(), max_corner.Y()));
}



double utilities::width(ignition::math::Box box){
	return std::abs(box.Max().X() - box.Min().X());
}

double utilities::height(ignition::math::Box box){
	return std::abs(box.Max().Y() - box.Min().Y());
}

double utilities::map(double val, double from_min, double from_max, double to_min, double to_max){
	double frac = (val - from_min)/(from_max-from_min);
	return to_min + frac*(to_max-to_min);
}

void utilities::print_vector(ignition::math::Vector3d vec){
	std::printf("(%f, %f, %f)\n", vec.X(), vec.Y(), vec.Z());
}

//returns the shortest normal vector between pos and one of the edges on the bounding box of entity
// will return the shortest corner distance if the normal does not exist 

ignition::math::Vector3d utilities::min_repulsive_vector(ignition::math::Vector3d pos, gazebo::physics::EntityPtr entity){
	std::vector<ignition::math::Line3d> edges = utilities::get_edges(entity);

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
		auto corners = utilities::get_corners(entity);
		for (auto corner: corners){
			if (pos.Distance(corner) < min_mag){
				min_mag = pos.Distance(corner);
				min_normal = pos-corner;
			}
		}
	}

	return min_normal;
}


bool utilities::contains(ignition::math::Box b1, ignition::math::Box b2){

	ignition::math::Vector3d min_corner = b2.Min();
	ignition::math::Vector3d max_corner = b2.Max();
	min_corner.Z() = 0;
	max_corner.Z() = 0;

	ignition::math::Vector3d bot_l = min_corner;
	ignition::math::Vector3d bot_r = ignition::math::Vector3d(max_corner.X(),min_corner.Y(),0);
	ignition::math::Vector3d top_l = ignition::math::Vector3d(min_corner.X(),max_corner.Y(),0);
	ignition::math::Vector3d top_r = max_corner;

	// check if each point lies within b1:

	if (utilities::inside_box(b1, bot_l) && utilities::inside_box(b1, bot_r) && utilities::inside_box(b1, top_l) && utilities::inside_box(b1, top_r)){
		return true;
	}

	return false;

}


utilities::Path::Path(){
	this->radius = 0.5;
}

utilities::Path::Path(double _radius){
	this->radius = _radius;
	
}

void utilities::Path::AddPoint(ignition::math::Vector3d _point){
	this->points.push_back(_point);
}