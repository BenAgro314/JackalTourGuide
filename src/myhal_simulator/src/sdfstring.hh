#ifndef SDFSTRING_HH
#define SDFSTRING_HH

#include <string>
#include <vector>
#include <gazebo/gazebo.hh>
#include <utility>

/*
class SDFString{

    public:

        std::string value;

        std::vector<std::shared_ptr<HeaderTag>> header_tags;

        std::vector<std::shared_ptr<DataTags>> data_tags;

        SDFString();

};
*/



class SDFTag{

    public: 

        std::string name;

        SDFTag(std::string _name);

        virtual std::string WriteTag(int pretabs);

};

class DataTag : public SDFTag{

    public: 

        std::string data;

        DataTag(std::string _name, std::string _data);

        std::string WriteTag(int pretabs);

};



class HeaderTag : public SDFTag{

    private:

        std::vector<std::shared_ptr<SDFTag>> sub_tags;

        std::vector<std::vector<std::string>> attributes;

    public:

        using SDFTag::SDFTag;

        std::string WriteTag(int pretabs);

        void AddAttribute(std::string title, std::string value);

        void AddSubtag(std::shared_ptr<SDFTag> _tag);

};

#endif