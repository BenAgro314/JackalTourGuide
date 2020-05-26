#include "sdfstring.hh"
#include <sstream> 
#include <iostream>

/*
SDFString::SDFString(){
    this->value = 
    "<sdf version ='1.6'>\
    </sdf>";
}
*/

SDFTag::SDFTag(std::string _name){
    
    this->name = _name;
}

std::string SDFTag::WriteTag(int pretabs){
    return "";
}


DataTag::DataTag(std::string _name, std::string _data):SDFTag(_name){
    this->data = _data;
}

std::string DataTag::WriteTag(int pretabs){
    std::stringstream tag;

    for (int i = 0; i<pretabs; i++){
        tag << "\t";
    }

    tag << "<" << this->name << ">" << this->data << "</" << this->name << ">\n";

    return tag.str();
    
}



void HeaderTag::AddAttribute(std::string title, std::string value){
    std::vector<std::string> new_attribute;
    new_attribute.push_back(title);
    new_attribute.push_back(value);

    this->attributes.push_back(new_attribute);
}

void HeaderTag::AddSubtag(std::shared_ptr<SDFTag> _tag){
    this->sub_tags.push_back(_tag);
}

std::string HeaderTag::WriteTag(int pretabs){
    std::stringstream tag;

    for (int i = 0; i<pretabs; i++){
        tag << "\t";
    }

    tag << "<" << this->name;

    for (std::vector<std::string> attribute: this->attributes){
        tag << " " << attribute[0] << "=\"" << attribute[1] << "\"";
    }
    tag << ">\n";

    for (std::shared_ptr<SDFTag> sub_tag: this->sub_tags){
        tag <<  sub_tag->WriteTag(pretabs+1);
    }

    for (int i = 0; i<pretabs; i++){
        tag << "\t";
    }

    tag << "</" << this->name << ">\n";

    return tag.str();
}
