#include "objects.hh"
#include <string>

using namespace objects;

Actor::Actor(std::string name, ignition::math::Pose3d initial_pose, std::string type): Object(name, initial_pose){
    this->type = type;
}

boost::shared_ptr<sdf::SDF> Actor::GetSDF(){
    
    boost::shared_ptr<sdf::SDF> sdf = boost::make_shared<sdf::SDF>();

    sdf->SetFromString( 
        "<sdf version ='1.6'>\
          <actor name ='actor'>\
          </actor>\
        </sdf>");

    auto actor = sdf->Root()->GetElement("actor");
    actor->GetAttribute("name")->SetFromString(name);
    actor->GetElement("pose")->Set(pose);

    actor->GetElement("skin")->GetElement("filename")->Set("model://actor/meshes/SKIN_man_blue_shirt.dae"); // TODO: generalize these filenames

    std::vector<math_utils::Tuple<std::string>> anims = {
        math_utils::Tuple<std::string>("walking","model://actor/meshes/ANIMATION_walking.dae"),
        math_utils::Tuple<std::string>("talking","model://actor/meshes/ANIMATION_talking_a.dae"),
        math_utils::Tuple<std::string>("standing","model://actor/meshes/ANIMATION_talking_a.dae"),
        math_utils::Tuple<std::string>("sitting","model://actor/meshes/ANIMATION_sitting.dae")};

    for (auto anim: anims){
        auto anim_el = actor->AddElement("animation");
        anim_el->GetAttribute("name")->SetFromString(anim.r);
        anim_el->GetElement("filename")->Set(anim.c);
        anim_el->GetElement("interpolate_x")->Set(true);
    }
    return sdf;
}

int Object::obj_count = 0;

Object::Object(std::string name, ignition::math::Pose3d pose):
name(name + "_" + std::to_string(obj_count)), pose(pose){
    this->obj_count++;
};

std::string Object::Name(){
    return name;
}

gazebo::physics::ModelPtr &Object::Model(){
    return world_model;
}

boost::shared_ptr<sdf::SDF> Object::GetSDF(){
    boost::shared_ptr<sdf::SDF> sdf = boost::make_shared<sdf::SDF>();
    return sdf;
}

void Object::AddToWorld(gazebo::physics::WorldPtr world){
    auto sdf = this->GetSDF();
    world->InsertModelSDF(*sdf);
}

Model::Model(std::string name, ignition::math::Pose3d pose, std::string uri): Object(name, pose){
    this->uri = uri;
}

boost::shared_ptr<sdf::SDF> Model::GetSDF(){
    boost::shared_ptr<sdf::SDF> sdf = boost::make_shared<sdf::SDF>();

    sdf->SetFromString( 
        "<sdf version ='1.6'>\
          <model name ='model'>\
          </model>\
        </sdf>");

    auto model = sdf->Root()->GetElement("model");
    model->GetAttribute("name")->SetFromString(name);
    model->GetElement("pose")->Set(pose);
    auto include = model->GetElement("include");
    include->GetElement("name")->Set(name+"_include");
    include->GetElement("uri")->Set(uri);
    
    return sdf;
}

Boxes::Boxes(std::string name): Object(name, ignition::math::Pose3d(0,0,0,0,0,0)){

}

void Boxes::AddBox(ignition::math::Box box){
    this->boxes.push_back(box);
}

boost::shared_ptr<sdf::SDF> Boxes::GetSDF(){
    int count = 0;

    boost::shared_ptr<sdf::SDF> sdf = boost::make_shared<sdf::SDF>();
    sdf->SetFromString(
       "<sdf version ='1.6'>\
          <model name ='box'>\
            <pose>0 0 0 0 0 0</pose>\
          </model>\
        </sdf>");

    auto model = sdf->Root()->GetElement("model");
    model->GetElement("static")->Set(true);
    model->GetAttribute("name")->SetFromString(name);

    for (auto box: this->boxes){

        auto center = (box.Min() + box.Max())/2;
        auto width = box.Max().X() - box.Min().X();
        auto depth = box.Max().Y() - box.Min().Y();
        auto height = box.Max().Z() - box.Min().Z();

        auto link = model->AddElement("link");
        link->GetAttribute("name")->SetFromString("l" + std::to_string(count));
        link->GetElement("pose")->Set(ignition::math::Pose3d(center, ignition::math::Quaterniond(0,0,0,0)));
        link->GetElement("collision")->GetElement("geometry")->GetElement("box")->GetElement("size")->Set(ignition::math::Vector3d(width,depth,height));
        link->GetElement("visual")->GetElement("geometry")->GetElement("box")->GetElement("size")->Set(ignition::math::Vector3d(width,depth,height));
        count++;
    }

    return sdf;
}
