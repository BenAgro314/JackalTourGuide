#include "maze.hh"

typedef ignition::math::Box ibox;
typedef ignition::math::Vector3d ivector;
typedef math_utils::Tuple<int> Tuple;
typedef ignition::math::Rand irand;

namespace dungeon{

void BSPDungeon::CreateChildren(){
    double w = bounds.Max().X() - bounds.Min().X();
    double l = bounds.Max().Y() - bounds.Min().Y();

    

    if ((bool) irand::IntUniform(0,1)){
        // vertical slice

        if (w<=2*min_w){
            this->CreateRoom();
            return;
        }

        auto rand_x = irand::DblUniform(bounds.Min().X() + min_w, bounds.Max().X()-min_w);
        // convert this to an index and back;
        auto ind = this->PosToIndicies(ivector(rand_x,0,0));
        rand_x = this->IndiciesToPos(ind).X();

        auto max_left = ivector(rand_x, bounds.Max().Y(),bounds.Max().Z());
        auto min_right = ivector(rand_x, bounds.Min().Y(), 0);

        this->child_a = boost::make_shared<BSPDungeon>(ibox(bounds.Min(), max_left), x_res, y_res, min_w,min_l,wall_w, hallway_w, min_room_w, min_room_l);
        this->child_b =  boost::make_shared<BSPDungeon>(ibox(min_right,bounds.Max()), x_res, y_res, min_w,min_l,wall_w, hallway_w, min_room_w, min_room_l);

        child_a->CreateChildren();
        child_b->CreateChildren();
    } else{
        // horizontal slice

        if (l <= 2*min_l){
            this->CreateRoom();
            return;
        }
        auto rand_y = irand::DblUniform(bounds.Min().Y() + min_l, bounds.Max().Y()-min_l);
        // convert this to an index and back;
        auto ind = this->PosToIndicies(ivector(0,rand_y,0));
        rand_y = this->IndiciesToPos(ind).Y();

        auto max_bot = ivector(bounds.Max().X(), rand_y,bounds.Max().Z());
        auto min_top = ivector(bounds.Min().X(), rand_y, 0);

        this->child_a = boost::make_shared<BSPDungeon>(ibox(bounds.Min(), max_bot), x_res, y_res, min_w,min_l,wall_w, hallway_w, min_room_w, min_room_l);
        this->child_b =  boost::make_shared<BSPDungeon>(ibox(min_top,bounds.Max()), x_res, y_res, min_w,min_l,wall_w, hallway_w, min_room_w, min_room_l);

        child_a->CreateChildren();
        child_b->CreateChildren();
    }
}

void BSPDungeon::ConnectChildren(){


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
    //std::printf("(%f, %f)\n", bounds.Min().X() + wall_w, bounds.Min().Y() + wall_w);
    auto min_inds = this->PosToIndicies(ivector(bounds.Min().X() + wall_w, bounds.Min().Y() + wall_w, 0));
    auto max_inds = this->PosToIndicies(ivector(bounds.Max().X()-wall_w, bounds.Max().Y() - wall_w,0));

    Tuple rand_min = min_inds;
    Tuple rand_max = max_inds;
   
    if (this->min_room_l >=0 && this->min_room_w >= 0){
       
        auto middle = (bounds.Max() + bounds.Min())/2;
        middle.Z() = 0;

        auto min_mid = middle;
        min_mid.X() -= this->min_room_w/2;
        min_mid.Y() -= this->min_room_l/2;
        auto max_mid = middle;
        max_mid.X() += this->min_room_w/2;
        max_mid.Y() += this->min_room_l/2;

        auto min_mid_i = this->PosToIndicies(min_mid);
        auto max_mid_i = this->PosToIndicies(max_mid);

        
        if (!(min_inds.r > min_mid_i.r || min_inds.c > min_mid_i.c || max_mid_i.r>max_inds.r || max_mid_i.c > max_inds.c)){
          
            rand_min = Tuple(irand::IntUniform(min_inds.r, min_mid_i.r), irand::IntUniform(min_inds.c, min_mid_i.c));
            rand_max = Tuple(irand::IntUniform(max_mid_i.r, max_inds.r), irand::IntUniform(max_mid_i .c, max_inds.c));
        }
    }

    //std::printf("Min: (%d, %d), Max: (%d, %d)\n", min_inds.r, min_inds.c, max_inds.r, max_inds.c);
    for (int r =0; r<this->rows; r++){
        for (int c =0; c<this->cols; c++){
            if (r < rand_max.r && r>=rand_min.r && c<rand_max.c && c>=rand_min.c){
                binary[r][c] = 0;
            } else{
                binary[r][c] = 1;
            }
        }
    }
    
}

void BSPDungeon::FillCells(){

    this->CreateChildren();

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

                    binary[inds.r][inds.c] = curr->binary[r][c];
           
                }
            }
            continue;
        }

        queue.push_back(curr->child_a);
        queue.push_back(curr->child_b);
    }

    this->ConnectChildren();


}

BSPDungeon::BSPDungeon(ibox bounds, double x_res, double y_res, double min_w, double min_l, double wall_w, double hallway_w, double min_room_w, double min_roow_l):
Grid(bounds, x_res, y_res){
    this->min_w = min_w;
    this->min_l = min_l;
    this->wall_w =wall_w;
    this->hallway_w = hallway_w;
    this->min_room_l = min_room_l;
    this->min_room_w = min_room_w;
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

}