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

ignition::math::Box Cell::GetBounds(){
    return this->bounds;
}

bool Cell::Filled(){
    return filled;
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

    while ((bounds.Max().Y() - r*this->y_res) > pos.Y()){
        r++;
    }

    while ((bounds.Min().X() + c*this->x_res) < pos.X()){
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
    auto boxes = objects::Boxes("boxes");

    for (auto row: cells){
        for (auto cell: row){
            if (cell->Filled()){
                boxes.AddBox(cell->GetBounds());
            }
        }
    }

    boxes.AddToWorld(world);
}

void Grid::FillCells(){
    for (auto row: cells){
        for (auto cell: row){
           
            cell->SetFill((bool) ignition::math::Rand::IntUniform(0,1));
            
        }
    }
}

BSPDungeon::BSPDungeon(ignition::math::Box bounds, double x_res, double y_res, int min_w, int min_l, int wall_w, int hallway_w):
Grid(bounds, x_res, y_res){
    this->min_w= min_w;
    this->min_l = min_l;
    this->wall_w = wall_w;
    this->hallway_w = hallway_w;
    // for (int r = 0; r < this->rows; ++r){
    //     for (int c =0; c<this->cols; ++c){
    //        cells[r][c]->SetFill(true);
    //     }
    // }
}

void BSPDungeon::CreateRooms(){
    
    
    
    if (rows <= min_l || cols <= min_w){

        // fill room
        this->FillRoom();

        return;
    }
 
 
    bool vert = (bool) ignition::math::Rand::IntUniform(0,1);
    if (vert){ // making a vertial cut

        int min_c = min_w + 2*wall_w;
        int max_c = cols-min_w - 2* wall_w;

        if (min_c >= max_c || min_c >= cols || max_c <=0){
            this->FillRoom();
            return;
        }

        int rand_col = ignition::math::Rand::IntUniform(min_c,max_c);

        auto left_max = IndiciesToPos(0,rand_col);
        left_max.Z() = bounds.Max().Z();
        auto right_min = IndiciesToPos(rows,rand_col);
        right_min.Z() = bounds.Min().Z();

        child_a = boost::make_shared<BSPDungeon>(ignition::math::Box(bounds.Min(),left_max), x_res, y_res, min_w, min_l, wall_w, hallway_w);
        child_b = boost::make_shared<BSPDungeon>(ignition::math::Box(right_min,bounds.Max()), x_res, y_res, min_w, min_l, wall_w, hallway_w);

        child_a->CreateRooms();
        child_b->CreateRooms();
    }else{

        int min_r = min_l + 2*wall_w;
        int max_r = rows-min_r;

        if (min_r >= max_r || min_r >= cols || max_r <=0){
            this->FillRoom();
            return;
        }

        int rand_row = ignition::math::Rand::IntUniform(min_r,max_r);

        auto bot_max = IndiciesToPos(rand_row, cols);
        bot_max.Z() = bounds.Max().Z();
        auto top_min = IndiciesToPos(rand_row,0);
        top_min.Z() = bounds.Min().Z();

        child_a = boost::make_shared<BSPDungeon>(ignition::math::Box(bounds.Min(),bot_max), x_res, y_res, min_w, min_l, wall_w, hallway_w);
        child_b = boost::make_shared<BSPDungeon>(ignition::math::Box(top_min,bounds.Max()), x_res, y_res, min_w, min_l, wall_w, hallway_w);

        child_a->CreateRooms();
        child_b->CreateRooms();

    }
}

void BSPDungeon::FillRoom(){
  
    // double rand_w = ignition::math::Rand::DblUniform(min_width, cols*x_res-2*x_res);
    // double rand_h = ignition::math::Rand::DblUniform(min_height, rows*y_res-2*y_res);


    // auto r_lim = (int)(rows*y_res-rand_h)/y_res; 
    // auto c_lim = (int)(cols*x_res-rand_w)/x_res;
    // std::cout << r_lim << " " << c_lim << std::endl;

    // int min_r = 2;
    // if (r_lim > 2){
    //     min_r = ignition::math::Rand::IntUniform(2,r_lim);
    // }
    // int min_c = 2;
    // if (c_lim > 2){
    //     min_c = ignition::math::Rand::IntUniform(2,c_lim);
    // } 
    // int max_r = min_r + rand_h/y_res;
    // int max_c = min_c + rand_w/x_res;

    //std::cout << this->bounds << std::endl;

    int min_r = wall_w;
    int min_c = wall_w;
    int max_r = rows-1-wall_w;
    int max_c = cols-1-wall_w;
 
    for (int r =0; r<this->rows; r++){
        for (int c =0; c<this->cols; c++){
            if (r <= max_r && r>=min_r && c<=max_c && c>=min_c){
                cells[r][c]->SetFill(false);
            } else{
                cells[r][c]->SetFill(true);
            }
        }
    }
    
}



void BSPDungeon::FillCells(){
    // traverse the tree using a queue
    // if it is a leaf node: add its cells to the boxes object

  
    this->CreateRooms();


    if (this->child_a == nullptr || this->child_b == nullptr){
        return;
    }

    std::vector<boost::shared_ptr<BSPDungeon>> queue;
    queue.push_back(this->child_a);
    queue.push_back(this->child_b);

    while (queue.size() > 0){
        auto curr= queue[0];
        queue.erase(queue.begin());
   
        if (curr->child_a == nullptr && curr->child_b == nullptr){
            
            for (int r= 0; r<curr->rows; r++){
                for (int c=0; c<curr->cols; c++){
                    auto pos = curr->IndiciesToPos(r,c);
                    auto inds = this->PosToIndicies(pos);
                    
                    // std::printf("child:(%d %d)->par:(%d %d)", r, c, inds.r, inds.c);
                    // std::cout << "pose: " << pos << std::endl;
                    this->cells[inds.r][inds.c]->SetFill(curr->cells[r][c]->Filled());
           
                }
            }
            continue;
        }

        queue.push_back(curr->child_a);
        queue.push_back(curr->child_b);
    }

    this->ConnectHallways();


}

void BSPDungeon::ConnectHallways(){

    

    if (this->child_a == nullptr || this->child_b==nullptr){
        return;
    }

    std::vector<boost::shared_ptr<BSPDungeon>> queue;
    queue.push_back(child_a);
    queue.push_back(child_b);

    int level = 1;

    while (queue.size() > 0){
        int n = queue.size();
        for (int i=0; i< n-1; i++){
            auto A = queue[i];
            auto B = queue[i+1];
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
                for (int c = std::min(A_cen_inds.c, B_cen_inds.c); c<std::max(A_cen_inds.c, B_cen_inds.c); c++){
                    std::cout << B_cen_inds.r << " " << c << std::endl;
                    // for (int r = std::max(this->wall_w,A_cen_inds.r-(int)std::floor(this->hallway_w/2)); r <= std::min(rows-wall_w, A_cen_inds.r+(int)std::floor(hallway_w/2)); r++){
                    //     std::cout << r << " " << c << std::endl;
                    //     this->cells[r][c]->SetFill(false);
                    // }
                    cells[A_cen_inds.r][c]->SetFill(false);
                    
                }
            }
             if (A_cen_inds.c == B_cen_inds.c){
                for (int r = std::min(A_cen_inds.r, B_cen_inds.r); r<std::max(A_cen_inds.r, B_cen_inds.r); r++){
                    std::cout << r << " " << A_cen_inds.c  << std::endl;
                    // for (int c = std::max(this->wall_w,  A_cen_inds.c-(int)std::floor(this->hallway_w/2)); c <= std::min(cols-wall_w, A_cen_inds.c+(int)std::floor(hallway_w/2)); c++){
                    //     std::cout << r << " " << c << std::endl;
                    //     this->cells[r][c]->SetFill(false);
                    // }
                    cells[r][A_cen_inds.c]->SetFill(false);
                    
                }
            }

        }
    }
}


}