#include "maze.hh"


namespace dungeon{

Cell::Cell(ignition::math::Box bounds, bool filled){
    this->bounds = bounds;
    this->filled = filled;
    if (this->filled){
        this->cost = 255;
    } else {
        this->cost = 1;
    }
}

void Cell::SetFill(bool filled){
    this->filled = filled;
    if (filled){
        this->cost = 255;
    } else {
        this->cost = 1;
    }
}

Grid::Grid(ignition::math::Box bounds, double x_res, double y_res){
    this->x_res = x_res;
    this->y_res = y_res;
    this->bounds = bounds;

    this->cols = (int) (bounds.Max().X() - bounds.Min().X())/x_res;
    
    this->rows = (int) (bounds.Max().Y() - bounds.Min().Y())/y_res;
 

    for (int r = 0; r < this->rows; ++r){
        std::vector<boost::shared_ptr<Cell>> new_row;
        
        for (int c =0; c<this->cols; ++c){
            auto top_left = this->IndiciesToPos(r,c);

            new_row.push_back(boost::make_shared<Cell>(ignition::math::Box(top_left.X(),top_left.Y()-y_res, top_left.Z(), top_left.X() + x_res, top_left.Y(), bounds.Max().Z())));
        }
        this->cells.push_back(new_row);
    }
}

void Cell::AddToWorld(gazebo::physics::WorldPtr world){
    
    if (this->filled){
        auto box = objects::Box(this->bounds);
        box.AddToWorld(world);
       
    }
}


Tuple Grid::PosToIndicies(ignition::math::Vector3d pos){
    int r = 0;
    int c = 0;

    while ((bounds.Max().Y() - r*this->y_res - this->y_res) > pos.Y()){
        r++;
    }

    while ((bounds.Min().X() + c*this->x_res + this->x_res) < pos.X()){
        c++;
    }

    return Tuple(r, c);
}

ignition::math::Vector3d Grid::IndiciesToPos(int r, int c){

    auto pos = ignition::math::Vector3d(bounds.Min().X() + c*x_res, bounds.Max().Y() - r*y_res, 0);

    return pos;
}

ignition::math::Vector3d Grid::IndiciesToPos(Tuple t){

    auto pos = ignition::math::Vector3d(bounds.Min().X() + t.c*x_res, bounds.Max().Y() - t.r*y_res, 0);
    
    return pos;
}

void Grid::AddToWorld(gazebo::physics::WorldPtr world){
    for (auto row: cells){
        for (auto cell: row){
          
            cell->AddToWorld(world);
        }
    }
}

void Grid::FillCells(){

  
  
    for (auto row: cells){
        for (auto cell: row){
           
            cell->SetFill((bool) ignition::math::Rand::IntUniform(0,1));
            
        }
    }
}


}