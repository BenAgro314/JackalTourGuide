#include "sdfstring.hh"
#include <sstream> 
#include <iostream>

SDFString::SDFString(){
    this->value = 
    "<sdf version ='1.6'>\
    </sdf>";
}


SDFTag::SDFTag(std::string _name){
    
    this->name = _name;
}

std::string SDFTag::WriteTag(){
    return "";
}



DataTag::DataTag(std::string _name, std::string _data):SDFTag(_name){
    this->data = _data;
}

std::string DataTag::WriteTag(){
    std::stringstream tag;

    tag << "<" << this->name << ">" << this->data << "</" << this->name << ">\n";

    return tag.str();
    
}



void HeaderTag::AddAttribute(std::string title, std::string value){
    std::vector<std::string> new_attribute;
    new_attribute.push_back(title);
    new_attribute.push_back(value);

    this->attributes.push_back(new_attribute);
}

void HeaderTag::AddSubtag(std::shared_ptr<DataTag> _tag){
    this->sub_tags.push_back(_tag);
}

std::string HeaderTag::WriteTag(){
    std::stringstream tag;

    tag << "<" << this->name;

    for (std::vector<std::string> attribute: this->attributes){
        tag << " " << attribute[0] << "=\"" << attribute[1] << "\"";
    }
    tag << ">\n";

    for (std::shared_ptr<DataTag> sub_tag: this->sub_tags){
        tag << "\t" << sub_tag->WriteTag();
    }

    tag << "</" << this->name << ">\n";

    return tag.str();
}
