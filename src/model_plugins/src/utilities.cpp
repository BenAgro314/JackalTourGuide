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

//returns true if the projection of pos falls within the bounds of edge. If it does, it stores the normal distance between the edge and pos in normal

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