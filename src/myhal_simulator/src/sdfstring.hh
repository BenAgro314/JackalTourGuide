#ifndef SDFSTRING_HH
#define SDFSTRING_HH

#include <string>
#include <vector>
#include <gazebo/gazebo.hh>
#include <utility>

class SDFString{

    public:

        SDFString();

        std::string value;

};




class SDFTag{

    public: 

        std::string name;

        SDFTag(std::string _name);

        virtual std::string WriteTag();

};

class DataTag : public SDFTag{

    public: 

        std::string data;

        DataTag(std::string _name, std::string _data);

        std::string WriteTag();

};



class HeaderTag : public SDFTag{

    private:

        std::vector<std::shared_ptr<DataTag>> sub_tags;

        std::vector<std::vector<std::string>> attributes;

    public:

        using SDFTag::SDFTag;

        std::string WriteTag();

        void AddAttribute(std::string title, std::string value);

        void AddSubtag(std::shared_ptr<DataTag> _tag);

};

#endif