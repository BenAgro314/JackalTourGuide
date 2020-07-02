#include "objects.hh"
#include <string>

using namespace objects;

int Object::obj_count = 0;

Object::Object(std::string name, ignition::math::Pose3d pose):
name(name + "_" + std::to_string(obj_count)), pose(pose){
    this->obj_count++;
};


boost::shared_ptr<sdf::SDF> Object::GetSDF(){
    boost::shared_ptr<sdf::SDF> sdf = boost::make_shared<sdf::SDF>();
    return sdf;
}

void Object::AddToWorld(gazebo::physics::WorldPtr world){
    return;    
}

Box::Box(ignition::math::Box box, std::string name):
Object(name, ignition::math::Pose3d(box.Min(), ignition::math::Quaterniond(0,0,0,0))){
    this->box = box;
}

boost::shared_ptr<sdf::SDF> Box::GetSDF(){

    auto center = (this->box.Min() + this->box.Max())/2;
    auto width = this->box.Max().X() - this->box.Min().X();
    auto depth = this->box.Max().Y() - this->box.Min().Y();
    auto height = this->box.Max().Z() - this->box.Min().Z();

    boost::shared_ptr<sdf::SDF> boxSDF = boost::make_shared<sdf::SDF>();
    boxSDF->SetFromString(
       "<sdf version ='1.6'>\
          <model name ='box'>\
            <pose>0 0 0 0 0 0</pose>\
            <link name ='link'>\
              <collision name ='collision'>\
                <geometry>\
                    <box>\
                        <size>0 0 0</size>\
                    </box>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                    <box>\
                        <size>0 0 0</size>\
                    </box>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>");
    
    boxSDF->Root()->GetElement("model")->GetAttribute("name")->SetFromString(this->name);
    boxSDF->Root()->GetElement("model")->GetElement("pose")->Set(ignition::math::Pose3d(center, ignition::math::Quaterniond(0,0,0,0)));
    boxSDF->Root()->GetElement("model")->GetElement("link")->GetElement("collision")->GetElement("geometry")->GetElement("box")->GetElement("size")->Set(ignition::math::Vector3d(width,depth,height));
    boxSDF->Root()->GetElement("model")->GetElement("link")->GetElement("visual")->GetElement("geometry")->GetElement("box")->GetElement("size")->Set(ignition::math::Vector3d(width,depth,height));
    

    return boxSDF;
}

void Box::AddToWorld(gazebo::physics::WorldPtr world){
    auto sdf = this->GetSDF();
    world->InsertModelSDF(*sdf);
}