#include "post_classifier.hh"
#include <dirent.h>


int main(int argc, char ** argv){

    if (argc ==1){
        std::cout << "must input bag name\n";
        return 0;
    }

    std::string user_name = "default";
    if (const char * user = std::getenv("USER")){
      user_name = user;
    } 

    std::string time_name = argv[1];

    Classifier classifier = Classifier(time_name, user_name);

    classifier.Load();

    return 0;
}

Classifier::Classifier(std::string filename, std::string username){
    this->filename = filename;
    this->username = username;
    this->filepath = "/home/" + this->username + "/Myhal_Simulation/simulated_runs/" + this->filename + "/";
}

void Classifier::Load(){
    std::cout << "Classifying files in: " << this->filepath << std::endl;

    // read trajectory


    happly::PLYData gt_pose = happly::PLYData(this->filepath + "gt_pose.ply");
    this->robot_trajectory = ReadTrajectory(gt_pose);

    // read frames

    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir ((this->filepath + "bag_names/").c_str())) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            printf ("%s\n", ent->d_name);
        }
        closedir (dir);
    } else {
        /* could not open directory */
        perror ("");
        std::cout << "HI\n";
    }


    // read static objects 


    // interpolate trajectory and offset frames 


    // iterate through the points of each frame, check collisions against static objects, classify points 
}