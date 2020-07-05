#include "maze.hh"


namespace dungeon{

void print_grid(std::vector<std::vector<int>> grid){
    for (int r =0; r<grid.size(); r++){
        for (int c= 0; c<grid.size(); c++){
            if (grid[r][c] == 1){
                std::cout << "#";
            }else{
                std::cout << ".";
            }
            
        }
        std::cout << std::endl;
    }
}

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
            new_row.back()->SetFill(true);
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

    // for (auto row: cells){
    //     for (auto cell: row){
    //         if (cell->Filled()){
    //             boxes.AddBox(cell->GetBounds());
    //         }
    //     }
    // }

    // boxes.AddToWorld(world);


    int num_on = 0;

    std::vector<std::vector<int>> grid;

    for (auto row: cells){
        std::vector<int> new_row;
        for (auto cell: row){
            if (cell->Filled()){
                new_row.push_back(1);
                num_on++;
            }else{
                new_row.push_back(0);
            }
        }
        grid.push_back(new_row);
    }

    print_grid(grid);
    while (num_on >0){
        
        boxes.AddBox(this->MaxRectangle(grid,num_on));
    }

    boxes.AddToWorld(world);

}

ignition::math::Box Grid::MaxRectangle(std::vector<std::vector<int>> &grid, int &num_on){ // returns the largest rectangle in the array that isn't included in taken and modifies taken to include that rectangle
   

    //iterate through the rows
    //keep a running histogram 
    //for each row compute the maxhistogram area, if it is a new global maximum, store the row index, area, and left and right indicies of the rectangle 
    //At the end, compute the resultant rectangle, and turn off the values in the grid

    std::vector<int> hist(grid[0].size(), 0);
    int max_area = 0;
    int left;
    int right;
    int max_row;

    for (int r =0; r<grid.size(); r++){
        for (int c = 0; c<grid[0].size(); c++){
            hist[c]=((grid[r][c] == 0) ? 0 : hist[c]+grid[r][c]);
        }
        int le,ri;
        int area =  this->MaxHistogramArea(hist, le, ri);
        if(max_area < area){
            max_area = area;
            max_row =r;
            left = le;
            right = ri;
        }

    }

    

    //std::printf("Min: max_row: %d, left: %d, right: %d, area: %d\n", max_row, left, right, max_area);
    auto min_corner = this->IndiciesToPos(max_row, left);
    min_corner.Y()-=y_res;
    int h = max_area/(right-left);
    auto max_corner = this->IndiciesToPos(max_row-h,right-1);
    max_corner.X() += x_res;
    max_corner.Y() -= y_res;

    min_corner.Z() = 0;
    max_corner.Z() = bounds.Max().Z();


    auto max_box = ignition::math::Box(min_corner, max_corner);
    //std::cout << max_box << std::endl;

    for (int r = max_row-h+1; r<=max_row;r++){
        for (int c=left; c<=right-1;c++){
            grid[r][c] =0;
            num_on--;
        }
    }
    

    return max_box;

} 

int Grid::MaxHistogramArea(std::vector<int> hist, int &l, int &r){
    std::vector<int> stack; //make this a vector of (bar height, bar index)

    int max_area = 0;
    int tp;
    int area_with_top;
    int i =0;

    while (i<hist.size()){
        if (stack.empty() || hist[stack.back()] <= hist[i]){
            stack.push_back(i++);
        }else{
            tp = stack.back();
            stack.pop_back();

            area_with_top = hist[tp]*(stack.empty() ? i:(i-stack.back()-1));

            if (max_area < area_with_top){
                max_area = area_with_top;
                l = (stack.empty() ? 0:(stack.back()+1));
                r = i;
            }

        }
    }

    while (stack.empty() == false) { 
        tp = stack.back(); 
        stack.pop_back(); 
      
        area_with_top = hist[tp]*(stack.empty() ? i:(i-stack.back()-1));
  
        if (max_area < area_with_top){
            max_area = area_with_top; 
            
            l = (stack.empty() ? 0:(stack.back()+1));
            r = i;
            
        } 
            
    } 
    
   
    return max_area; 
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

        int min_c = min_w + wall_w;
        int max_c = cols-min_w - wall_w;

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

        int min_r = min_l + wall_w;
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
                for (int c = std::min(A_cen_inds.c, B_cen_inds.c); c<std::max(A_cen_inds.c, B_cen_inds.c); c++){

                    if (hallway_w %2 ==0){
                        for (int r = std::max(this->wall_w,A_cen_inds.r-(int)std::floor(this->hallway_w/2)); r < std::min(rows-wall_w, A_cen_inds.r+(int)std::floor(hallway_w/2)); r++){

                            this->cells[r][c]->SetFill(false);
                        }
                    } else  {
                        for (int r = std::max(this->wall_w,A_cen_inds.r-(int)std::floor(this->hallway_w/2)); r <= std::min(rows-wall_w, A_cen_inds.r+(int)std::floor(hallway_w/2)); r++){

                            this->cells[r][c]->SetFill(false);
                        }
                    }

                    
                }
            }
             if (A_cen_inds.c == B_cen_inds.c){
                for (int r = std::min(A_cen_inds.r, B_cen_inds.r); r<std::max(A_cen_inds.r, B_cen_inds.r); r++){
                    if (hallway_w % 2 == 0){
                        for (int c = std::max(this->wall_w,  A_cen_inds.c-(int)std::floor(this->hallway_w/2)); c < std::min(cols-wall_w, A_cen_inds.c+(int)std::floor(hallway_w/2)); c++){
                            this->cells[r][c]->SetFill(false);
                        }
                    } else{
                        for (int c = std::max(this->wall_w,  A_cen_inds.c-(int)std::floor(this->hallway_w/2)); c <= std::min(cols-wall_w, A_cen_inds.c+(int)std::floor(hallway_w/2)); c++){
                            this->cells[r][c]->SetFill(false);
                        }
                    }

                }
            }

        }
    }
}


}