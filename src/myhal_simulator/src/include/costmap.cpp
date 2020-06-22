#include "costmap.hh"

Costmap::Costmap(ignition::math::Box boundary, double resolution){
    this->boundary = boundary;
    this->resolution = resolution;

    this->top_left = ignition::math::Vector3d(boundary.Min().X(), boundary.Max().Y(), 0);
    this->width = boundary.Max().X() - boundary.Min().X();
    this->height = boundary.Max().Y() - boundary.Min().Y();

    this->cols = this->width/this->resolution;
    this->rows = this->height/this->resolution;
    
    for (int r = 0; r <this->rows; ++r){
        std::vector<int> new_row;
        std::vector<double> new_row2;
        for (int c = 0; c< this->cols; ++c){
            new_row.push_back(1);
            new_row2.push_back(10e9);
        }
        this->costmap.push_back(new_row);
        this->integration_field.push_back(new_row2);
    }

    this->last_path = this->costmap;

}

void Costmap::AddObject(ignition::math::Box object){
    object.Min().Z() = 0;
    object.Max().Z() = 0;

    auto tl = ignition::math::Vector3d(object.Min().X(), object.Max().Y(), 0);
    auto br = ignition::math::Vector3d(object.Max().X(), object.Min().Y(), 0);

    if (tl.X() >= this->boundary.Max().X() || tl.Y() <= this->boundary.Min().Y() || br.X() <= this->boundary.Min().X() || br.Y() >= this->boundary.Max().Y()){
        return;
    }

    int min_r, min_c;
    int max_r, max_c;

    this->PosToIndicies(tl, min_r, min_c);
    
    this->PosToIndicies(br, max_r, max_c);

    //std::printf("tl: (%f, %f), br: (%f, %f), min: (%d, %d), max (%d, %d)\n", tl.X(), tl.Y(), br.X(), br.Y(), min_r, min_c, max_r, max_c);

    for (int r = min_r; r<=max_r; ++r){
        for(int c = min_c; c<=max_c; ++c){
            this->costmap[r][c] = 255;
        }
    }

    this->last_path = this->costmap;

}

std::string Costmap::ToString(){
    std::stringstream out;

    for (int r = 0; r<this->rows; r++){
        for (int c= 0; c<this->cols; c++){
            if (this->costmap[r][c] == 1){
                out << "*";
            } else{
                out << "▇";
            }
        }
        out << "\n";
    }


    return out.str();
}

std::string Costmap::PathString(){
    std::stringstream out;

    for (int r = 0; r<this->rows; r++){
        for (int c= 0; c<this->cols; c++){
            if (this->last_path[r][c] == 1){
                out << " ";
            } else if (this->last_path[r][c] == -1) {
                out << "X";
            } else {
                out << "▇";
            }
        }
        out << "\n";
    }

    return out.str();
}

bool Costmap::FindPath(ignition::math::Vector3d start, ignition::math::Vector3d end,  std::vector<ignition::math::Vector3d> &path){
    if (!this->Integrate(end)){
        return false;
    }


    if (start.X() < this->top_left.X() || start.X() > this->boundary.Max().X() || start.Y() > this->top_left.Y() || start.Y() < this->boundary.Min().Y()){
        return false;
    }

    

    path.push_back(start);

    int end_r, end_c, start_r, start_c;

    this->PosToIndicies(start, start_r, start_c);
    this->PosToIndicies(end, end_r, end_c);

    if (this->integration_field[start_r][start_c] == 10e9){
        return false;
    }

    auto curr_pos = start;
    std::vector<int> curr_ind = {start_r, start_c};
    this->last_path[curr_ind[0]][curr_ind[1]] = -1;

    while (curr_ind[0] != end_r || curr_ind[1] != end_c){

        // iterate over neighbours to find minimum cost path
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


        if (!found){
            return false;
        } 
            

        ignition::math::Vector3d pos1, pos2;
        this->IndiciesToPos(pos1, curr_ind[0], curr_ind[1]);
        this->IndiciesToPos(pos2, min_n[0], min_n[1]);
        
        auto offset = pos2 - pos1;
        curr_pos += offset;
        path.push_back(curr_pos);
        curr_ind = min_n;


        this->last_path[curr_ind[0]][curr_ind[1]] = -1;

    }

    return true;
}

bool Costmap::Integrate(ignition::math::Vector3d goal){

    for (int r = 0; r <this->rows; ++r){
        
        for (int c = 0; c< this->cols; ++c){
            this->integration_field[r][c] = 10e9;
        }
        
    }
    
    if (goal.X() < this->top_left.X() || goal.X() > this->boundary.Max().X() || goal.Y() > this->top_left.Y() || goal.Y() < this->boundary.Min().Y()){
        return false;
    }

    int goal_r, goal_c;
    this->PosToIndicies(goal, goal_r, goal_c);

    this->integration_field[goal_r][goal_c] = 0;

    std::vector<std::vector<int>> open_list;
    open_list.push_back({goal_r, goal_c});

    while (open_list.size() > 0){
        auto curr_ind = open_list.front();
        open_list.erase(open_list.begin());

        auto neighbours = this->GetNeighbours(curr_ind);

        for (auto n: neighbours){
            auto n_cost = ((double) this->costmap[n[0]][n[1]]) + this->integration_field[curr_ind[0]][curr_ind[1]];
            if (n_cost < this->integration_field[n[0]][n[1]] && this->costmap[n[0]][n[1]] < 255){
                this->integration_field[n[0]][n[1]] = n_cost;
                if (std::find(open_list.begin(), open_list.end(), n) == open_list.end()){
                    open_list.push_back(n);
                }
            }

            
        }

    }

    return true;

}

std::vector<std::vector<int>> Costmap::GetNeighbours(std::vector<int> curr_ind, bool diag){
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

bool Costmap::PosToIndicies(ignition::math::Vector3d pos, int &r, int &c){
    
    r = 0;
    c = 0;

    while ((this->top_left.Y() - r*this->resolution - this->resolution) > pos.Y()){
        r++;
    }

    while ((this->top_left.X() + c*this->resolution + this->resolution) < pos.X()){
        c++;
    }


    /*
    r = (int) (this->boundary.Max().Y() - pos.Y())/this->resolution;
    if (r >= this->rows){
        r = this->rows-1;
    }
    if (r < 0){
        r = 0;
    }
    c = (int) (pos.X() -this->boundary.Min().X())/this->resolution;
    if (c >= this->cols){
        c = this->cols-1;
    }
    if (c < 0){
        c = 0;
    }
    */

    return utilities::inside_box(this->boundary, pos, true);
}

bool Costmap::IndiciesToPos(ignition::math::Vector3d &pos, int r, int c){
    pos = ignition::math::Vector3d(this->boundary.Min().X() + c*this->resolution, this->boundary.Max().Y() - r*this->resolution, 0);
    return ((r>=0 && r < this->rows) && (c>=0 && c < this->cols));
}