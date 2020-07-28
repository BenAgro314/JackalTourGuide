#include "world_entities.hh"
#include "utilities.hh"
#include <algorithm>

using namespace myhal;

///Model

int Model::num_models = 0;

Model::Model(std::string _name, ignition::math::Pose3d _pose, std::string _model_file, double _width, double _length)
{
    this->name = _name + "_" + std::to_string(num_models); //this ensures name uniqueness
    this->pose = _pose;
    this->model_file = _model_file;
    num_models++;

    this->corners.push_back(ignition::math::Vector3d(_pose.Pos().X() + _width / 2, _pose.Pos().Y() + _length / 2, 0)); //top right
    this->corners.push_back(ignition::math::Vector3d(_pose.Pos().X() - _width / 2, _pose.Pos().Y() + _length / 2, 0)); //top left
    this->corners.push_back(ignition::math::Vector3d(_pose.Pos().X() - _width / 2, _pose.Pos().Y() - _length / 2, 0)); //bot left
    this->corners.push_back(ignition::math::Vector3d(_pose.Pos().X() + _width / 2, _pose.Pos().Y() - _length / 2, 0)); // bot right
}

void Model::AddPlugin(std::shared_ptr<SDFPlugin> plugin)
{
    this->plugins.push_back(plugin);
}

std::string Model::CreateSDF()
{
    return "";
}

void Model::AddToWorld(std::string &world_string)
{
    std::string sdf = this->CreateSDF();
    world_string += sdf;
}

ignition::math::Box Model::GetCollisionBox()
{
    // return the smallest box that this object can be contained within

    double min_x = 10e6;
    double max_x = -10e6;
    double min_y = 10e6;
    double max_y = -10e6;
    for (auto corner : this->corners)
    {
        if (corner.X() > max_x)
        {
            max_x = corner.X();
        }
        if (corner.X() < min_x)
        {
            min_x = corner.X();
        }
        if (corner.Y() > max_y)
        {
            max_y = corner.Y();
        }
        if (corner.Y() < min_y)
        {
            min_y = corner.Y();
        }
    }

    return ignition::math::Box(ignition::math::Vector3d(min_x, min_y, 0), ignition::math::Vector3d(max_x, max_y, 10));
}

double Model::GetWidth(){
    auto box = this->GetCollisionBox();
    return box.Max().X() - box.Min().X();
}

double Model::GetLength(){
    auto box = this->GetCollisionBox();
    return box.Max().Y() - box.Min().Y();
}

bool Model::DoesCollide(std::shared_ptr<Model> other)
{

    auto other_box = other->GetCollisionBox();
    auto this_box = this->GetCollisionBox();

    if (other_box.Intersects(this_box))
    {
        return true;
    }

    //redundancy check: TODO: remove
    double minx = other_box.Min().X();
    double miny = other_box.Min().Y();
    double maxx = other_box.Max().X();
    double maxy = other_box.Max().Y();

    if (this->pose.Pos().X() > std::min(minx, maxx) && this->pose.Pos().X() < std::max(minx, maxx))
    {
        if (this->pose.Pos().Y() > std::min(miny, maxy) && this->pose.Pos().Y() < std::max(miny, maxy))
        {
            return true;
        }
    }

    return false;
}

void Model::Reposition(double new_x, double new_y)
{
    double x_shift = new_x - this->pose.Pos().X();
    double y_shift = new_y - this->pose.Pos().Y();

    this->pose.Pos().X() = new_x;
    this->pose.Pos().Y() = new_y;

    for (int i = 0; i < (int)this->corners.size(); ++i)
    {
        this->corners[i].X() += x_shift;
        this->corners[i].Y() += y_shift;
    }
}

void Model::RotateClockwise(double angle)
{
    // rotate and find new corners

    auto rotation = ignition::math::Quaterniond(0, 0, angle);

    for (int i = 0; i < (int)this->corners.size(); ++i)
    {
        auto corner_vector = this->corners[i] - this->pose.Pos();
        corner_vector = rotation.RotateVector(corner_vector);
        this->corners[i] = corner_vector + this->pose.Pos();
    }

    auto current_yaw = this->pose.Rot().Yaw();
    this->pose.Rot() = ignition::math::Quaterniond(this->pose.Rot().Roll(), this->pose.Rot().Pitch(), current_yaw + angle);
}

///IncludeModel

std::string IncludeModel::CreateSDF()
{

    //std::shared_ptr<HeaderTag> model = std::make_shared<HeaderTag>("model");
    //model->AddAttribute("name", this->name);

    std::shared_ptr<HeaderTag> include = std::make_shared<HeaderTag>("include");

    std::shared_ptr<DataTag> name = std::make_shared<DataTag>("name", this->name); //+ "_include");

    std::string pose_string = std::to_string(this->pose.Pos().X()) + " " + std::to_string(this->pose.Pos().Y()) + " " + std::to_string(this->pose.Pos().Z()) + " " + std::to_string(this->pose.Rot().Roll()) + " " + std::to_string(this->pose.Rot().Pitch()) + " " + std::to_string(this->pose.Rot().Yaw());
    std::shared_ptr<DataTag> pose_tag = std::make_shared<DataTag>("pose", pose_string);

    std::shared_ptr<DataTag> uri = std::make_shared<DataTag>("uri", this->model_file);

    include->AddSubtag(name);
    include->AddSubtag(pose_tag);
    include->AddSubtag(uri);

    //model->AddSubtag(include);

    std::stringstream sdf;

    sdf << include->WriteTag(2);

    return sdf.str();
}

///Actor

void Actor::AddAnimation(std::shared_ptr<SDFAnimation> animation)
{
    this->animations.push_back(animation);
}

std::string Actor::CreateSDF()
{

    //make appropriate tags:

    //skin
    std::shared_ptr<DataTag> s_file = std::make_shared<DataTag>("filename", this->model_file);
    std::shared_ptr<HeaderTag> s_header = std::make_shared<HeaderTag>("skin");
    s_header->AddSubtag(s_file);

    //pose
    std::string pose_string = std::to_string(this->pose.Pos().X()) + " " + std::to_string(this->pose.Pos().Y()) + " " + std::to_string(this->pose.Pos().Z()) + " " + std::to_string(this->pose.Rot().Roll()) + " " + std::to_string(this->pose.Rot().Pitch()) + " " + std::to_string(this->pose.Rot().Yaw());
    std::shared_ptr<DataTag> pose_tag = std::make_shared<DataTag>("pose", pose_string);

    std::shared_ptr<HeaderTag> actor = std::make_shared<HeaderTag>("actor");

    actor->AddAttribute("name", this->name);
    actor->AddSubtag(pose_tag);
    actor->AddSubtag(s_header);

    for (std::shared_ptr<SDFAnimation> animation : this->animations)
    {
        actor->AddSubtag(animation);
    }

    for (std::shared_ptr<SDFPlugin> plugin : this->plugins)
    {
        actor->AddSubtag(plugin);
    }

    //write to stream

    std::stringstream sdf;

    sdf << actor->WriteTag(2);

    //std::cout << sdf.str() << std::endl;
    return sdf.str();
}

///BoundaryBox

BoundaryBox::BoundaryBox(double _x, double _y, double _width, double _length)
    : Model("box", ignition::math::Pose3d(_x, _y, -0.5, 0, 0, 0), "", _width, _length)
{
    this->width = _width;
    this->length = _length;
}

std::string BoundaryBox::CreateSDF()
{
    std::shared_ptr<HeaderTag> model = std::make_shared<HeaderTag>("model");
    model->AddAttribute("name", this->name);

    std::string pose_string = std::to_string(this->pose.Pos().X()) + " " + std::to_string(this->pose.Pos().Y()) + " " + std::to_string(this->pose.Pos().Z()) + " " + std::to_string(this->pose.Rot().Roll()) + " " + std::to_string(this->pose.Rot().Pitch()) + " " + std::to_string(this->pose.Rot().Yaw());
    std::shared_ptr<DataTag> pose_tag = std::make_shared<DataTag>("pose", pose_string);

    model->AddSubtag(pose_tag);

    std::shared_ptr<DataTag> static_tag = std::make_shared<DataTag>("static", "true");

    model->AddSubtag(static_tag);

    std::shared_ptr<HeaderTag> link = std::make_shared<HeaderTag>("link");
    link->AddAttribute("name", this->name + "_link");

    std::shared_ptr<HeaderTag> collision = std::make_shared<HeaderTag>("collision");
    collision->AddAttribute("name", this->name + "_collision");

    std::shared_ptr<HeaderTag> geometry = std::make_shared<HeaderTag>("geometry");
    std::shared_ptr<HeaderTag> box = std::make_shared<HeaderTag>("box");

    std::string size_string = std::to_string(this->width) + " " + std::to_string(this->length) + " 1";
    std::shared_ptr<DataTag> size = std::make_shared<DataTag>("size", size_string);

    box->AddSubtag(size);
    geometry->AddSubtag(box);
    collision->AddSubtag(geometry);
    link->AddSubtag(collision);
    model->AddSubtag(link);

    std::stringstream sdf;
    sdf << model->WriteTag(2);
    return sdf.str();
}

///Camera

Camera::Camera(std::string name, ignition::math::Pose3d _pose, std::string path): Model(name, _pose, "", 0.1, 0.1){
    this->name = name;
    if (path == ""){
        this->save = false;
    }else{
        this->save = true;
    }
    this->filepath = path;
}

std::string Camera::CreateSDF(){

    std::shared_ptr<HeaderTag> model = std::make_shared<HeaderTag>("model");
    model->AddAttribute("name", this->name);

    std::string pose_string = std::to_string(this->pose.Pos().X()) + " " + std::to_string(this->pose.Pos().Y()) + " " + std::to_string(this->pose.Pos().Z()) + " " + std::to_string(this->pose.Rot().Roll()) + " " + std::to_string(this->pose.Rot().Pitch()) + " " + std::to_string(this->pose.Rot().Yaw());

    std::shared_ptr<DataTag> pose_tag = std::make_shared<DataTag>("pose", pose_string);
    std::shared_ptr<DataTag> static_tag = std::make_shared<DataTag>("static", "true");
    model->AddSubtag(pose_tag);
    model->AddSubtag(static_tag);

    std::shared_ptr<HeaderTag> link = std::make_shared<HeaderTag>("link");
    link->AddAttribute("name", this->name + "_link");

    auto sensor = std::make_shared<HeaderTag>("sensor");
    sensor->AddAttribute("name", this->name); 
    sensor->AddAttribute("type", "camera"); 

    auto camera = std::make_shared<HeaderTag>("camera");
    auto save = std::make_shared<HeaderTag>("save");

    auto hfov = std::make_shared<DataTag>("horizontal_fov", "1.047");
    camera->AddSubtag(hfov);

    auto img = std::make_shared<HeaderTag>("image");
    auto w = std::make_shared<DataTag>("width","1920");
    auto h = std::make_shared<DataTag>("height","1080");
    img->AddSubtag(w);
    img->AddSubtag(h);
    camera->AddSubtag(img);

    auto clip = std::make_shared<HeaderTag>("clip");
    auto near = std::make_shared<DataTag>("near","0.1");
    auto far = std::make_shared<DataTag>("far","100");
    clip->AddSubtag(near);
    clip->AddSubtag(far);
    camera->AddSubtag(clip);

    sensor->AddSubtag(camera);

    auto plugin = std::make_shared<HeaderTag>("plugin");
    plugin->AddAttribute("name", "camera_controller");
    plugin->AddAttribute("filename", "libcamera_controller.so");

    auto path = std::make_shared<DataTag>("filepath", this->filepath + this->name + "/");
    plugin->AddSubtag(path);

    auto always_on = std::make_shared<DataTag>("always_on", "1");
    auto update_rate = std::make_shared<DataTag>("update_rate", "24");
    sensor->AddSubtag(plugin);
    sensor->AddSubtag(always_on);
    sensor->AddSubtag(update_rate);

    link->AddSubtag(sensor);
    model->AddSubtag(link);

    std::stringstream sdf;
    sdf << model->WriteTag(2);
    return sdf.str();

}


///TableGroup

TableGroup::TableGroup(std::shared_ptr<Model> _table_model, std::shared_ptr<Model> _chair_model, int _num_chairs, double _rotation_angle){
    this->table_model = _table_model;
    this->chair_model = _chair_model;
    this->num_chairs = std::min(_num_chairs,4); //TODO: allow more chairs in the future

    this->rotation_angle = _rotation_angle;
    this->table_model->RotateClockwise(_rotation_angle);
    auto corners = table_model->corners;

    auto pos = table_model->pose.Pos();
    pos.Z() = 0;

    int start = ignition::math::Rand::IntUniform(0,3);

    // make sitter plugin:

    for (int i = start; i < start+num_chairs; i++){
        std::shared_ptr<Model> new_chair = std::make_shared<IncludeModel>("chair", table_model->pose, this->chair_model->model_file, this->chair_model->GetWidth(), this->chair_model->GetWidth());
        new_chair->RotateClockwise(ignition::math::Rand::DblUniform(0,6.28));

        auto table_edge = ignition::math::Line3d(corners[(i+1)%(corners.size())], corners[i%(corners.size())]);
       
        // find normal from tables center to edge 
        ignition::math::Vector3d normal;
    
        
        if (utilities::get_normal_to_edge(pos, table_edge, normal)){
            normal*=-1;
            auto res_pos = pos+normal;
            normal.Normalize();
            normal*= (new_chair->GetWidth()/1.5); //TODO: fix this
            res_pos+=normal;
            new_chair->Reposition(res_pos.X(), res_pos.Y());

            this->chairs.push_back(new_chair);
        }
    }

}

///Room

Room::Room(double x_min, double y_min, double x_max, double y_max, std::vector<ignition::math::Box> walls, std::vector<ignition::math::Pose3d> route, bool _enclosed = false)
{
    this->boundary = ignition::math::Box(ignition::math::Vector3d(x_min, y_min, 0), ignition::math::Vector3d(x_max, y_max, 10));
    this->enclosed = _enclosed;

    if (this->enclosed)
    {
        //create boundary box
        std::shared_ptr<BoundaryBox> bot = std::make_shared<BoundaryBox>((x_min + x_max) / 2, y_min - 0.125, x_max - x_min, 0.25);
        std::shared_ptr<BoundaryBox> top = std::make_shared<BoundaryBox>((x_min + x_max) / 2, y_max + 0.125, x_max - x_min, 0.25);
        std::shared_ptr<BoundaryBox> left = std::make_shared<BoundaryBox>(x_min - 0.125, (y_min + y_max) / 2, 0.25, y_max - y_min + 0.5);
        std::shared_ptr<BoundaryBox> right = std::make_shared<BoundaryBox>(x_max + 0.125, (y_min + y_max) / 2, 0.25, y_max - y_min + 0.5);

        this->models.push_back(bot);
        this->models.push_back(top);
        this->models.push_back(left);
        this->models.push_back(right);
    }

    this->walls = walls;
    this->route = route;
}

bool Room::AddModel(std::shared_ptr<Model> model)
{   
    double x_min = this->boundary.Min().X();
    double y_min = this->boundary.Min().Y();
    double x_max = this->boundary.Max().X();
    double y_max = this->boundary.Max().Y();
    double width = model->GetWidth();
    double length = model->GetLength();
    
    for (auto other : this->models)
    {
        if (model->DoesCollide(other))
        {
            return false;
        }

        auto model_box = model->GetCollisionBox();
        for (auto box: this->walls){
            if (box.Intersects(model_box)){
                return false;
            }
        }

        for (auto pose: this->route){
            if(utilities::inside_box(model_box, pose.Pos(), true)){
                return false;
            }
        }

        if (this->enclosed){
            if (model->pose.Pos().X() < (x_min+(width/2))){
                return false;
            }

            if (model->pose.Pos().X() > (x_max-(width/2))){
                return false;
            }

            if (model->pose.Pos().Y() > (y_max-(length/2))){
                return false;
            }

            if (model->pose.Pos().Y() < (y_min+(length/2))){
                return false;
            }
        }
    }
    
    this->models.push_back(model);
    return true;
}

bool Room::AddModelRandomly(std::shared_ptr<Model> model)
{

    //update our list of collision links to consider (those that are part of this room and haven't already been added)

    int iterations = 0;
    bool found = false;

    auto col_box = model->GetCollisionBox();
    double width = model->GetWidth();
    double length = model->GetLength();

    while (iterations < 1000 && !found)
    {

        //for now we will just guess randomly in the box. TODO: use a more nuanced method to choose guess point

        double x_min = this->boundary.Min().X();
        double y_min = this->boundary.Min().Y();
        double x_max = this->boundary.Max().X();
        double y_max = this->boundary.Max().Y();

        model->Reposition(ignition::math::Rand::DblUniform(x_min + (width / 2), x_max - (width / 2)), ignition::math::Rand::DblUniform(y_min + (length / 2), y_max - (length / 2)));

        found = true;

        //if the position is invald

        for (auto other : this->models)
        {
            if (model->DoesCollide(other))
            {
                found = false;
            }
        }

        iterations++;
    }

    if (found)
    {
        this->AddModel(model);
    }
    else
    {
        std::cout << "no place found" << std::endl;
    }

    return found;
}

void Room::AddToWorld(std::string &world_string)
{
    for (std::shared_ptr<Model> model : this->models)
    {
        model->AddToWorld(world_string);
    }
}

double Room::Area()
{
    double x_min = this->boundary.Min().X();
    double y_min = this->boundary.Min().Y();
    double x_max = this->boundary.Max().X();
    double y_max = this->boundary.Max().Y();

    return (x_max - x_min) * (y_max - y_min);
}
