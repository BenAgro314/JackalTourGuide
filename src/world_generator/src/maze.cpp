#include "maze.hh"

typedef ignition::math::Box ibox;
typedef ignition::math::Vector3d ivector;
typedef math_utils::Tuple<int> Tuple;
typedef ignition::math::Rand irand;
typedef boost::shared_ptr<dungeon::BSPDungeon> DungeonPtr;

namespace dungeon{


void BSPDungeon::CreateConnections(){
    // for all leave dugneons, add them to the graph connections
    // if there is a line of sight from one dungeon center to the other, add and edge between them weighted by the triangular distance (ie base + height of hypotenous)
    // the id of the dungeon vertex will be denoted by its index in the vector leaves
    
    std::vector<DungeonPtr> leaves_list;
    for (auto it = leaves.begin(); it != leaves.end(); it++){
        auto cen = (it->first->room_bounds.Max() - it->first->room_bounds.Min())/2;
        std::cout << cen << std::endl;
        std::string name = std::to_string(cen.X()) + " " + std::to_string(cen.Y());
        connections.AddVertex(name,it->first, it->second);
        leaves_list.push_back(it->first);
    }

    
    for (auto parent: leaves_list){

        for (auto n: leaves_list){
            if (leaves[parent] == leaves[n]){
                continue; // don't connect a dungeon to itself 
            }

            // check if there is a line of sight that doesn't go through another room 

            auto par_cen = (parent->room_bounds.Max() - parent->room_bounds.Min())/2;
            par_cen.Z() = 0;

            auto n_cen = (n->room_bounds.Max() - n->room_bounds.Min())/2;
            n_cen.Z() = 0;

            auto ray = ignition::math::Line3d(par_cen, n_cen);

            bool status = true;

            for (auto other: leaves_list){
                if (leaves[other] == leaves[n] || leaves[other] == leaves[parent]){ // dont check for intersection with parent or n
                    continue;
                }

                if (std::get<0>((other->room_bounds).Intersect(ray))){
                    status = false;
                }
            }

            double tri_dist = std::abs(par_cen.X() - n_cen.X()) + std::abs(par_cen.Y() - n_cen.Y());

            if (status){
                connections.AddEdge(std::make_pair(leaves[parent],leaves[n]), ray, tri_dist);
            }
        }
    }

    std::cout << connections <<std::endl;
}

void BSPDungeon::SetDensity(double prob){
    dense_prob = prob;
    if (dense_prob >1){
        dense_prob = 1;
    } else if (dense_prob < 0){
        dense_prob = 0;
    }
}

void BSPDungeon::Slice(bool vert){
    if (vert){
        auto rand_x = irand::DblUniform(bounds.Min().X() + room_specs->width, bounds.Max().X()-room_specs->width);
        // convert this to an index and back;
        auto ind = this->PosToIndicies(ivector(rand_x,0,0));
        rand_x = this->IndiciesToPos(ind).X();

        rand_x+= ((bool)irand::IntUniform(0,1)) ? (x_res/2) : (-1*x_res/2); // ensure we are on a grid edge

        auto max_left = ivector(rand_x, bounds.Max().Y(),bounds.Max().Z());
        auto min_right = ivector(rand_x, bounds.Min().Y(), 0);

        this->child_a = boost::make_shared<BSPDungeon>(ibox(bounds.Min(), max_left), x_res, y_res, room_specs, hallway_w);
        this->child_b =  boost::make_shared<BSPDungeon>(ibox(min_right,bounds.Max()), x_res, y_res, room_specs, hallway_w);
        child_a->SetDensity(dense_prob);
        child_b->SetDensity(dense_prob);
    } else{
        auto rand_y = irand::DblUniform(bounds.Min().Y() + room_specs->length, bounds.Max().Y()-room_specs->length);
        // convert this to an index and back;
        auto ind = this->PosToIndicies(ivector(0,rand_y,0));
        rand_y = this->IndiciesToPos(ind).Y();

        rand_y += ((bool)irand::IntUniform(0,1)) ? (y_res/2) : (-1*y_res/2);

        auto max_bot = ivector(bounds.Max().X(), rand_y,bounds.Max().Z());
        auto min_top = ivector(bounds.Min().X(), rand_y, 0);

        this->child_a = boost::make_shared<BSPDungeon>(ibox(bounds.Min(), max_bot), x_res, y_res, room_specs, hallway_w);
        this->child_b =  boost::make_shared<BSPDungeon>(ibox(min_top,bounds.Max()), x_res, y_res, room_specs, hallway_w);
        child_a->SetDensity(dense_prob);
        child_b->SetDensity(dense_prob);
    }
}

void BSPDungeon::CreateChildren(){
    double w = bounds.Max().X() - bounds.Min().X();
    double l = bounds.Max().Y() - bounds.Min().Y();

    // if can't be sliced vertically: slice horizontally
    // if can't be sliced horizontally: slice veritcally
    // if can't be sliced either way: create room
    // if can be sliced both ways: choose random direction

    bool vert = w>2*room_specs->width;
    bool hor =  l>2*room_specs->length;
    
    if (irand::DblUniform(0,1) < dense_prob){
        if (vert && !hor){
            this->Slice(true);
        } else if (!vert && hor){
            this->Slice(false);
        } else if (!vert && !hor){
            this->CreateRoom();
            return;
        } else {
            this->Slice((bool)irand::IntUniform(0,1));
        }
    } else{
        if ((bool) irand::IntUniform(0,1)){
            if (w<=2*room_specs->width){
                this->CreateRoom();
                return;
            }
            this->Slice(true);
        } else{
            if (l <= 2*room_specs->length){
                this->CreateRoom();
                return;
            }
            this->Slice(false);
        }
    }
    
    child_a->CreateChildren();
    child_b->CreateChildren();
}

void BSPDungeon::ConnectChildren(){


    if (this->child_a == nullptr || this->child_b==nullptr){
        return;
    }

    std::vector<DungeonPtr> queue;
    queue.push_back(child_a);
    queue.push_back(child_b);

    int level = 1;

    while (queue.size() > 0){
        int n = queue.size();
        //std::cout << n << std::endl;
        for (int i=0; i< n-1; i+=2){
            auto A = queue[0];
            auto B = queue[1];
            queue.erase(queue.begin());
            queue.erase(queue.begin());

            if (A->child_a != nullptr){
                queue.push_back(A->child_a);
                queue.push_back(A->child_b);
            }
            if (B->child_a != nullptr){
                queue.push_back(B->child_a);
                queue.push_back(B->child_b);
            }

            // obtain the centers of A and B

            auto A_cen = (A->bounds.Max() + A->bounds.Min())/2;
            A_cen.Z() = 0;
            auto B_cen = (B->bounds.Max() + B->bounds.Min())/2;
            B_cen.Z() = 0;

            // carve out a hallway along A_cen -> B_cen

            auto A_cen_inds = this->PosToIndicies(A_cen);
            auto B_cen_inds = this->PosToIndicies(B_cen);

            if (A_cen_inds.r == B_cen_inds.r){
                int min_r = A_cen_inds.r - std::round(hallway_w/2);
                int max_r = A_cen_inds.r + std::round(hallway_w/2);
                for (int c = std::min(A_cen_inds.c, B_cen_inds.c); c<=std::max(A_cen_inds.c, B_cen_inds.c); c++){
                    for (int r = std::max(0,min_r); r<=std::min(rows-1,max_r); r++){
                        binary[r][c] = 0;
                    }
                    
                }
            } else if (A_cen_inds.c == B_cen_inds.c){
                int min_c = A_cen_inds.c - std::round(hallway_w/2);
                int max_c = A_cen_inds.c + std::round(hallway_w/2);
                for (int r = std::min(A_cen_inds.r, B_cen_inds.r); r<=std::max(A_cen_inds.r, B_cen_inds.r); r++){
                    for (int c = std::max(0,min_c); c<=std::min(cols-1,max_c); c++){
                        binary[r][c] = 0;
                    }
                }
            }

        }
    }
}

void BSPDungeon::CreateRoom(){

    auto min_inds = this->PosToIndicies(ivector(bounds.Min().X() + room_specs->min_wall_width, bounds.Min().Y() + room_specs->min_wall_width, 0));
    auto max_inds = this->PosToIndicies(ivector(bounds.Max().X()-room_specs->min_wall_width, bounds.Max().Y() - room_specs->min_wall_width,0));
    auto min_mid_i = this->PosToIndicies(ivector(bounds.Min().X() + room_specs->max_wall_width, bounds.Min().Y() + room_specs->max_wall_width, 0));
    auto max_mid_i = this->PosToIndicies(ivector(bounds.Max().X() - room_specs->max_wall_width, bounds.Max().Y() - room_specs->max_wall_width, 0));
   
    Tuple rand_min = min_inds;
    Tuple rand_max = max_inds;

    if (!(min_inds.r >= min_mid_i.r || min_inds.c >= min_mid_i.c || max_mid_i.r>=max_inds.r || max_mid_i.c >= max_inds.c)){
        rand_min = Tuple(irand::IntUniform(min_inds.r, min_mid_i.r), irand::IntUniform(min_inds.c, min_mid_i.c));
        rand_max = Tuple(irand::IntUniform(max_mid_i.r, max_inds.r), irand::IntUniform(max_mid_i .c, max_inds.c));
    }

    
    for (int r =0; r<this->rows; r++){
        for (int c =0; c<this->cols; c++){
            if (r < rand_max.r && r>=rand_min.r && c<rand_max.c && c>=rand_min.c){
                binary[r][c] = 0;
            } else{
                binary[r][c] = 1;
            }
        }
    }

    room_bounds.Min() = this->IndiciesToPos(rand_min);
    room_bounds.Min().Z() = bounds.Min().Z();
    room_bounds.Min().Y() -= y_res/2;
    room_bounds.Min().X() -= x_res/2;
    room_bounds.Max() = this->IndiciesToPos(rand_max);
    room_bounds.Max().Z() = bounds.Max().Z();
    room_bounds.Max().X() -= x_res/2;
    room_bounds.Max().Y() -= y_res/2;

    //std::cout << room_bounds << std::endl;
    //std::printf("BOUNDS: w: %f, h: %f, ROOM: w: %f, h: %f\n", bounds.Max().X()-bounds.Min().X(), bounds.Max().Y() - bounds.Min().Y(), room_bounds.Max().X() - room_bounds.Min().X(), room_bounds.Max().Y() - room_bounds.Min().Y());
}

void BSPDungeon::FillCells(){

    this->CreateChildren();

    if (this->child_a == nullptr || this->child_b == nullptr){
        return;
    }

    std::vector<DungeonPtr> queue;
    queue.push_back(this->child_a);
    queue.push_back(this->child_b);

    int ind =0;

    while (queue.size() > 0){
        auto curr= queue[0];
        queue.erase(queue.begin());
   
        if (curr->child_a == nullptr && curr->child_b == nullptr){
            //leaves.push_back(curr);
            leaves[curr] = ind++;
            for (int r= 0; r<curr->rows; r++){
                for (int c=0; c<curr->cols; c++){
                    auto pos = curr->IndiciesToPos(r,c);
                    auto inds = this->PosToIndicies(pos);

                    binary[inds.r][inds.c] = curr->binary[r][c];
           
                }
            }
            continue;
        }

        queue.push_back(curr->child_a);
        queue.push_back(curr->child_b);
    }

    this->ConnectChildren();
    //this->CreateConnections();
    //std::cout << children.size() << std::endl;

}

BSPDungeon::BSPDungeon(ignition::math::Box bounds, double x_res, double y_res, boost::shared_ptr<RoomInfo> room_specs, double hallway_width):
Grid(bounds, x_res, y_res){

    this->hallway_w = hallway_width;
    this->room_specs = room_specs;
}
    

void Grid::AddToWorld(gazebo::physics::WorldPtr world){

    int num_on = 0;

    std::vector<std::vector<int>> binary_copy;

    for (auto row: binary){
        std::vector<int> new_row;
        for (auto cell: row){
            new_row.push_back(cell);
            if (cell == 1){
                num_on++;
            }
        }
        binary_copy.push_back(new_row);
    }

    while (num_on >0){
        auto ind_box = math_utils::MaxRectangle(binary_copy);
        auto min_i = ind_box.r;
        auto max_i = ind_box.c; 

        for (int r = min_i.r; r<=max_i.r; r++){
            for (int c = min_i.c; c<=max_i.c; c++){
                num_on--;
                binary_copy[r][c] = 0;
            }
        }

        ivector min_v = this->IndiciesToPos(min_i);
        ivector max_v = this->IndiciesToPos(max_i);

        min_v.X() -= x_res/2;
        min_v.Y() -= y_res/2;
        min_v.Z() = 0;
        max_v.X() += x_res/2;
        max_v.Y() +=y_res/2;
        max_v.Z() = bounds.Max().Z();
        
        boxes->AddBox(ibox(min_v, max_v));
    }

    boxes->AddToWorld(world);
}


ivector Grid::IndiciesToPos(int r, int c){
    //return the center of the cell in row r and column c
 
    double x = bounds.Min().X() + (c*x_res) + x_res/2;
    double y = bounds.Min().Y() + (r*y_res) + y_res/2;

    return ivector(x,y,0);
}

ivector Grid::IndiciesToPos(Tuple t){
    //return the center of the cell in row r and column c
    double x = bounds.Min().X() + (t.c*x_res) + x_res/2;
    double y = bounds.Min().Y() + (t.r*y_res) + y_res/2;

    return ivector(x,y,0);
}

Tuple Grid::PosToIndicies(ivector pos){
    // we define pos as lying in row r and column c iff
    // box.Min().X() + x_res*c <= pos.X() < box.Min().X() + x_res*(c+1);
    // box.Min().Y() + y_res*r <= pos.Y() < box.Min().Y() + y_res*(r+1);
    // unless we are on the outer edge (ie pos.X() == box.Max().X() || pos.Y() == box.Max().Y())

    int c = std::floor((pos.X() - bounds.Min().X())/x_res);
    if (pos.X() == bounds.Max().X()){
        c = cols -1;
    } 
    
    int r = std::floor((pos.Y() - bounds.Min().Y())/y_res);
    if (pos.Y() == bounds.Max().Y()){
        r = rows-1;
    }

    return Tuple(r,c);
}

void Grid::Fill(){

    for (int r = 0; r< rows; r++){
        for (int c =0; c<cols; c++){
            binary[r][c] = irand::IntUniform(0,1);
        }
    }

    //binary[0][0]=1;
}

void Grid::ToString(){
    for (int r=0; r< rows; r++){
        for (int c= 0; c<cols; c++){
            if (binary[r][c] == 1){
                std::cout << "#";
            }else{
                std::cout << ".";
            }
        }
        std::cout << std::endl;
    }
}

Grid::Grid(ibox bounds, double x_res, double y_res){
    boxes = boost::make_shared<objects::Boxes>("boxes");
    rows = std::round((bounds.Max().Y()-bounds.Min().Y())/y_res);
    cols = std::round((bounds.Max().X()-bounds.Min().X())/x_res);
    this->x_res = x_res;
    this->y_res = y_res;
    this->bounds = bounds;
    for (int r = 0; r<rows;r++){
        std::vector<int> b_row;
        for (int c =0; c<cols; c++){
            b_row.push_back(0);
        }
        binary.push_back(b_row);
    }
}

RoomInfo::RoomInfo(double width, double length, double min_wall_width, double max_wall_width){
    this->width = width;
    this->length = length;
    this->min_wall_width = min_wall_width;
    this->max_wall_width = max_wall_width;
}

}
