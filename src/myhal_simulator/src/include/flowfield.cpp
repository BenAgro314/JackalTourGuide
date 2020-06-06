#include "flowfield.hh"

FlowField::FlowField(ignition::math::Vector3d top_left, double width, double height, double resolution):
field((int) height/resolution, std::vector<ignition::math::Vector3d>((int) width/resolution, ignition::math::Vector3d(0,0,0))){
    



    this->rect = ignition::math::Box(ignition::math::Vector3d(top_left.X(),top_left.Y() - height,0), ignition::math::Vector3d(top_left.X() + width, top_left.Y(),0));
    this->resolution = resolution;

    this->cols = (int) width/this->resolution;
    this->rows = (int) height/this->resolution;

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

std::vector<std::vector<int>> FlowField::GetNeighbours(std::vector<int> curr_ind, bool diag){
    std::vector<std::vector<int>> res;

    if (curr_ind[0] > 0){ // we can return top
        res.push_back({curr_ind[0]-1, curr_ind[1]});
    }

    if (curr_ind[1] > 0){ // we can return left
        res.push_back({curr_ind[0], curr_ind[1]-1});
    }

    if (curr_ind[0] < this->rows-1){ // we can return bot
        res.push_back({curr_ind[0]+1, curr_ind[1]});
    }

    if (curr_ind[1] < this->cols-1){ // we can return right
        res.push_back({curr_ind[0], curr_ind[1]+1});
    }

    if (diag){
        if (curr_ind[0] > 0 && curr_ind[1] > 0){ // we can return top left
            res.push_back({curr_ind[0]-1,curr_ind[1]-1});
        }

        if (curr_ind[0] > 0 && curr_ind[1] <this->cols-1){ // we can return bottom left
            res.push_back({curr_ind[0]-1,curr_ind[1]+1});
        }

        if (curr_ind[0] < this->rows-1 && curr_ind[1] > 0){ // we can return top right
            res.push_back({curr_ind[0]+1,curr_ind[1]-1});
        }

        if (curr_ind[0] < this->rows-1 && curr_ind[1] < this->cols-1){ // we can return bottom right
            res.push_back({curr_ind[0]+1,curr_ind[1]+1});
        }
    }

    return res;
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

        double min_z = std::min(box.Min().Z(), box.Max().Z());
        if (min_z > 1.5){
            continue;
        }

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

void FlowField::IntegrationField(double x, double y){

    for (int r = 0; r < this->rows; r++){
        std::vector<double> new_row;
        this->integration_field.push_back(new_row);
        for (int c =0; c < this->cols; c++){
            this->integration_field[r].push_back(10e9);
        }
    }
  
    int t_row, t_col;

    this->PosToIndicies(ignition::math::Vector3d(x,y,0), t_row, t_col);

    this->integration_field[t_row][t_col] = 0;

    std::vector<std::vector<int>> open_list;
    open_list.push_back({t_row, t_col});

    while (open_list.size() > 0){
        auto curr_ind = open_list.front();
        open_list.erase(open_list.begin());

        auto neighbours = this->GetNeighbours(curr_ind);

        for (auto n: neighbours){
            auto n_cost = this->costmap[n[0]][n[1]] + this->integration_field[curr_ind[0]][curr_ind[1]];
            if (n_cost < this->integration_field[n[0]][n[1]] && this->costmap[n[0]][n[1]] < 255){
                this->integration_field[n[0]][n[1]] = n_cost;
                if (std::find(open_list.begin(), open_list.end(), n) == open_list.end()){
                    open_list.push_back(n);
                }
            }

            
        }

    }

}

void FlowField::TargetInit(std::vector<gazebo::physics::EntityPtr> collision_entities, ignition::math::Vector3d target){
    this->CostMap(collision_entities);
    this->SetTarget(target);
    
}

void FlowField::SetTarget(ignition::math::Vector3d target){
    this->IntegrationField(target.X(), target.Y());

    for (int r = 0; r<this->rows; r++){
        for (int c= 0; c<this->cols; c++){
            
            auto curr_ind = {r, c};
            auto neighbours = this->GetNeighbours(curr_ind, true);

            std::vector<int> min_n;
            double min_val = 10e9;
            
            bool found = false;
            for (auto n : neighbours){
               
                double val = this->integration_field[n[0]][n[1]];
            
                if (val != 10e9){
                    found = true;
                }
              
                if (val < min_val){
                    min_val = val;
                    min_n = n;
                }
            }
       
            ignition::math::Vector3d dir, this_pos, n_pos;

            if (!found){
                dir = ignition::math::Vector3d(0,0,0);
            } else {

                this_pos = this->IndiciesToPos(r, c);
                n_pos = this->IndiciesToPos(min_n[0], min_n[1]);
                dir = n_pos-this_pos;
                dir = dir.Normalize();
            }
            this->field[r][c] = dir;
       
        }

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