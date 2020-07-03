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

BSPDungeon::BSPDungeon(ignition::math::Box bounds, double x_res, double y_res, double room_area, double min_width, double min_height):
Grid(bounds, x_res, y_res){
    this->room_area= room_area;
    this->min_height = min_height;
    this->min_width = min_width;
}

void BSPDungeon::CreateRooms(){
    double area = this->rows*y_res*this->cols*x_res;

    if (area <= room_area || this->rows*y_res <= min_height || this->cols*x_res <= min_width){

        // fill room
        this->FillRoom();

        return;
    }

    bool vert = (bool) ignition::math::Rand::IntUniform(0,1);
    if (vert){ // making a vertial cut

        int min_c = min_width/x_res;
        int max_c = cols-1-min_c;

        if (min_c >= max_c || min_c >= cols || max_c <=0){
            this->FillRoom();
            return;
        }

        int rand_col = ignition::math::Rand::IntUniform(min_c,max_c);

        auto left_max = IndiciesToPos(0,rand_col);
        left_max.Z() = bounds.Max().Z();
        auto right_min = IndiciesToPos(rows-1,rand_col);
        right_min.Z() = bounds.Min().Z();

        child_a = boost::make_shared<BSPDungeon>(ignition::math::Box(bounds.Min(),left_max), x_res, y_res, room_area, min_width, min_height);
        child_b = boost::make_shared<BSPDungeon>(ignition::math::Box(right_min,bounds.Max()), x_res, y_res, room_area, min_width, min_height);

        child_a->CreateRooms();
        child_b->CreateRooms();
    }else{

        int min_r = min_height/y_res;
        int max_r = rows-1-min_r;

        if (min_r >= max_r || min_r >= cols || max_r <=0){
            this->FillRoom();
            return;
        }

        int rand_row = ignition::math::Rand::IntUniform(min_r,max_r);

        auto bot_max = IndiciesToPos(rand_row, cols-1);
        bot_max.Z() = bounds.Max().Z();
        auto top_min = IndiciesToPos(rand_row,0);
        top_min.Z() = bounds.Min().Z();

        child_a = boost::make_shared<BSPDungeon>(ignition::math::Box(bounds.Min(),bot_max), x_res, y_res, room_area, min_width, min_height);
        child_b = boost::make_shared<BSPDungeon>(ignition::math::Box(top_min,bounds.Max()), x_res, y_res, room_area, min_width, min_height);

        child_a->CreateRooms();
        child_b->CreateRooms();

    }
}

void BSPDungeon::FillRoom(){

    double rand_w = ignition::math::Rand::DblUniform(min_width, cols*x_res-2*x_res);
    double rand_h = ignition::math::Rand::DblUniform(min_height, rows*y_res-2*y_res);

    int min_r = ignition::math::Rand::IntUniform(1,(int)(rows*y_res-rand_h)/y_res);
    int min_c = ignition::math::Rand::IntUniform(1,(int)(cols*x_res-rand_w)/x_res);
    int max_r = min_r + rand_h/y_res;
    int max_c = min_c + rand_w/x_res;

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
    // traverse the tree using a stack
    // if it is a leaf node: add its cells to the boxes object


    this->CreateRooms();
    if (this->child_a == nullptr || this->child_b == nullptr){
        return;
    }

    std::vector<boost::shared_ptr<BSPDungeon>> stack;
    stack.push_back(this->child_a);
    stack.push_back(this->child_b);

    while (stack.size() > 0){
        auto child = stack[0];
        stack.erase(stack.begin());

        if (child->child_a == nullptr || child->child_b == nullptr){
            for (int r= 0; r<child->rows; r++){
                for (int c=0; c<child->cols; c++){
                    auto pos = child->IndiciesToPos(r,c);
                    auto inds = this->PosToIndicies(pos);
                    this->cells[inds.r][inds.c]->SetFill(child->cells[r][c]->Filled());
                }
            }
            continue;
        }

        stack.push_back(this->child_a);
        stack.push_back(this->child_b);
    }


}




}