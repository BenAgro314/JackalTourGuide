#include "world_entities.hh"

using namespace myhal;

int Model::num_models = 0;

Model::Model(std::string _name, ignition::math::Pose3d _pose){
    this->name = _name + "_" + std::to_string(num_models); //this ensures name uniqueness 
    this->pose = _pose;
    num_models++;
}


void Model::AddPlugin(std::shared_ptr<SDFPlugin>  plugin){
    this->plugins.push_back(plugin);
}

std::string Model::CreateSDF(){

    return "";
}

Actor::Actor(std::string _name, ignition::math::Pose3d _pose, std::string _skin_file): Model(_name, _pose){
    this->skin_file = _skin_file;
}


void Actor::AddAnimation(std::shared_ptr<SDFAnimation> animation){
    this->animations.push_back(animation);
}

std::string Actor::CreateSDF(){

    //make appropriate tags:

    //skin
    std::shared_ptr<DataTag> s_file = std::make_shared<DataTag>("filename", this->skin_file);
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

    sdf<< "<sdf version ='1.6'>\n";

    sdf << actor->WriteTag(1);

    sdf << "</sdf>\n";

    return sdf.str();
}