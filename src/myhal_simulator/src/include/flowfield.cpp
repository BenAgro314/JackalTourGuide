#include "flowfield.hh"

FlowField::FlowField(ignition::math::Vector3d top_left, double width, double height, double resolution){
   
    this->rect = ignition::math::Box(ignition::math::Vector3d(top_left.X(),top_left.Y() - height,0), ignition::math::Vector3d(top_left.X() + width, top_left.Y(),0));
    this->resolution = resolution;

    this->cols = (int) width/this->resolution;
    this->rows = (int) height/this->resolution;

    this->PerlinInit();
}

ignition::math::Vector3d FlowField::IndiciesToPos(int r, int c){
    return ignition::math::Vector3d(this->rect.Min().X() + c*this->resolution, this->rect.Max().Y() - r*this->resolution, 0);
}

bool FlowField::PosToIndicies(ignition::math::Vector3d pos, int &r, int &c){
    

    r = (int) (this->rect.Max().Y() - pos.Y())/this->resolution;
    if (r >= this->rows){
        r = this->rows-1;
    }
    if (r < 0){
        r = 0;
    }
    c = (int) (pos.X() -this->rect.Min().X())/this->resolution;
    if (c >= this->cols){
        c = this->cols-1;
    }
    if (c < 0){
        c = 0;
    }

    return utilities::inside_box(this->rect, pos, true);

}

void FlowField::CostMap(std::vector<gazebo::physics::EntityPtr> collision_entities){
    // for each collision box, turn off cells in the flow field that contain it

    for (int r = 0; r < this->rows; ++r){
        std::vector<double> new_row;
        this->costmap.push_back(new_row);
        for (int c= 0; c< this->cols; ++c){
            this->costmap[r].push_back(1);
        }
    }

    for (auto object: collision_entities){
        auto box = object->BoundingBox();

        int min_col, min_row;
        this->PosToIndicies(ignition::math::Vector3d(box.Min().X(), box.Max().Y(), 0), min_row, min_col);

        int max_col, max_row;
        this->PosToIndicies(ignition::math::Vector3d(box.Max().X(), box.Min().Y(), 0), max_row, max_col);

        for (int r = min_row; r <= max_row; r++){
            for (int c = min_col; c<= max_col; c++){
                this->costmap[r][c] = 255;
            }
        }
        
    }

}

void FlowField::PrintField(){
    for (int r = 0; r < this->rows; r++){
        for (int c = 0; c< this->cols; c++){
            std::printf("%.1f ", this->costmap[r][c]); 
        }
        std::printf("\n");
    }
}

void FlowField::PerlinInit(){

    Perlin p;
    for (int r = 0; r<this->rows; r++){
        std::vector<ignition::math::Vector3d> row;
        
        for (int c= 0; c<this->cols; c++){
            auto pos = this->IndiciesToPos(r,c);
            double x = utilities::map(pos.X(), this->rect.Min().X(),this->rect.Max().X(), 0, 1);
            double y = utilities::map(pos.Y(), this->rect.Min().Y(),this->rect.Max().Y(), 0, 1);
          
            
            double theta = utilities::map(p.noise(x, y, 0), -0.5, 0.5, 0, 6.28);
       
            auto dir = ignition::math::Vector3d(1,0,0);
            auto rot = ignition::math::Quaterniond(0,0,theta);
            dir = rot.RotateVector(dir);
            row.push_back(dir);

        }
       
        this->field.push_back(row);
    }
}

bool FlowField::Lookup(ignition::math::Vector3d pos, ignition::math::Vector3d &res){
    int row_num, col_num;
    if(!this->PosToIndicies(pos, row_num, col_num)){
        return false;
    }
    res = this->field[row_num][col_num];
    return true;
}