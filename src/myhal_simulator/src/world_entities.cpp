#include "world_entities.hh"

using namespace myhal;

///Model

int Model::num_models = 0;

Model::Model(std::string _name, ignition::math::Pose3d _pose, std::string _model_file, double _width, double _length){
    this->name = _name + "_" + std::to_string(num_models); //this ensures name uniqueness 
    this->pose = _pose;
    this->model_file = _model_file;
    num_models++;
    auto min = ignition::math::Vector3d(_pose.Pos().X()-_width/2, _pose.Pos().Y()-_length/2, 0);
    auto max = ignition::math::Vector3d(_pose.Pos().X()+_width/2, _pose.Pos().Y()+_length/2, 10);
    this->box = ignition::math::Box(min,max);
}

void Model::AddPlugin(std::shared_ptr<SDFPlugin>  plugin){
    this->plugins.push_back(plugin);
}

std::string Model::CreateSDF(){
    return "";
}

void Model::AddToWorld(std::string &world_string){
    //std::cout << "here1\n";
    std::string sdf = this->CreateSDF();
    //std::cout << "here3\n";
    world_string+= sdf; 
}



///IncludeModel

std::string IncludeModel::CreateSDF(){

    
    std::shared_ptr<HeaderTag> model = std::make_shared<HeaderTag>("model");
    model->AddAttribute("name", this->name);

    std::shared_ptr<HeaderTag> include = std::make_shared<HeaderTag>("include");

    std::shared_ptr<DataTag> name = std::make_shared<DataTag>("name", this->name + "_include");

    std::string pose_string = std::to_string(this->pose.Pos().X()) + " " + std::to_string(this->pose.Pos().Y()) + " " + std::to_string(this->pose.Pos().Z()) + " " + std::to_string(this->pose.Rot().Roll()) + " " + std::to_string(this->pose.Rot().Pitch()) + " " + std::to_string(this->pose.Rot().Yaw());
    std::shared_ptr<DataTag> pose_tag = std::make_shared<DataTag>("pose", pose_string);

    std::shared_ptr<DataTag> uri = std::make_shared<DataTag>("uri", this->model_file);

    include->AddSubtag(name);
    include->AddSubtag(pose_tag);
    include->AddSubtag(uri);

    model->AddSubtag(include);

    std::stringstream sdf;

    sdf << model->WriteTag(2);
    
    return sdf.str();
}

///Actor

void Actor::AddAnimation(std::shared_ptr<SDFAnimation> animation){
    this->animations.push_back(animation);
}

std::string Actor::CreateSDF(){
   
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


    for (std::shared_ptr<SDFAnimation> animation : this->animations){
        actor->AddSubtag(animation);
    }

  

    for (std::shared_ptr<SDFPlugin> plugin : this->plugins){
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
: Model("box", ignition::math::Pose3d(_x,_y,-0.5, 0,0,0), "", _width, _length){
    this->width = _width;
    this->length = _length;
}

std::string BoundaryBox::CreateSDF(){
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

///ModelGroup

// ModelGroup::ModelGroup(std::string _name, ignition::math::Pose3d _pose, std::string _model_file){
//     this->center = std::make_shared<IncludeModel>(_name, _pose, _model_file);
//     this->group.push_back(this->center);
// }

// ignition::math::Pose3d ModelGroup::GetCenterPose(){
//     return this->center->pose;
// }

// void ModelGroup::AddObject(std::string _name, ignition::math::Pose3d _pose, std::string _model_file){
//     this->group.push_back(std::make_shared<IncludeModel>(_name, _pose, _model_file));
// }

///Room

Room::Room(double x_min, double y_min, double x_max, double y_max, bool _enclosed = false){
    this->boundary = ignition::math::Box(ignition::math::Vector3d(x_min,y_min,0), ignition::math::Vector3d(x_max,y_max,10));
    this->enclosed = _enclosed;
     
    if (this->enclosed){
        //create boundary box
        std::shared_ptr<BoundaryBox> bot = std::make_shared<BoundaryBox>((x_min+x_max)/2,y_min-0.125,x_max-x_min,0.25);
        std::shared_ptr<BoundaryBox> top = std::make_shared<BoundaryBox>((x_min+x_max)/2,y_max+0.125,x_max-x_min,0.25);
        std::shared_ptr<BoundaryBox> left = std::make_shared<BoundaryBox>(x_min-0.125,(y_min+y_max)/2,0.25,y_max-y_min+0.5);
        std::shared_ptr<BoundaryBox> right = std::make_shared<BoundaryBox>(x_max+0.125,(y_min+y_max)/2,0.25,y_max-y_min+0.5);

        this->AddModel(bot);
        this->AddModel(top);
        this->AddModel(left);
        this->AddModel(right);
    }

}

void Room::AddModel(std::shared_ptr<Model> model){
    this->models.push_back(model);
}


bool Room::AddModelRandomly(std::shared_ptr<Model> model){
   
    //update our list of collision links to consider (those that are part of this room and haven't already been added)

    int iterations = 0;
    bool found = false;

    
    double width = std::abs(model->box.Max().X() - model->box.Min().X());
    double length = std::abs(model->box.Max().Y() - model->box.Min().Y());
    ignition::math::Pose3d res_pose = ignition::math::Pose3d(0,0,model->pose.Pos().Z(), model->pose.Rot().Roll(), model->pose.Rot().Pitch(), model->pose.Rot().Yaw()); // we want to maintain the models current height and orientation

    while (iterations < 1000 && !found){
       
        //for now we will just guess randomly in the box. TODO: use a more nuanced method to choose guess point

        double x_min = this->boundary.Min().X();
        double y_min = this->boundary.Min().Y();
        double x_max = this->boundary.Max().X();
        double y_max = this->boundary.Max().Y();

        res_pose.Pos().X() = ignition::math::Rand::DblUniform(x_min+(width/2), x_max-(width/2));
        res_pose.Pos().Y() = ignition::math::Rand::DblUniform(y_min+(length/2), y_max-(length/2));

        model->box.Min() += ignition::math::Vector3d(res_pose.Pos().X(), res_pose.Pos().Y(), 0);
        model->box.Max() += ignition::math::Vector3d(res_pose.Pos().X(), res_pose.Pos().Y(), 0);

        found = true;

        //if the position is invald 

        for (auto other: this->models){
            if (other->box.Intersects(model->box)){
                found = false;
            }
        }

        iterations++;
    }

    if (found){
        model->pose = res_pose;
        this->AddModel(model);
    } else{
        std::cout << "no place found" << std::endl;
    }

    return found;
}


void Room::AddToWorld(std::string &world_string){
    for (std::shared_ptr<Model> model : this->models){
        model->AddToWorld(world_string);
    }
}

double Room::Area(){
    double x_min = this->boundary.Min().X();
    double y_min = this->boundary.Min().Y();
    double x_max = this->boundary.Max().X();
    double y_max = this->boundary.Max().Y();

    return (x_max-x_min)*(y_max- y_min);
}