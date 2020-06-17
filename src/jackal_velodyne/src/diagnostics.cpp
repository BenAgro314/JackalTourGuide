#include "diagnostics.hh"


int main(int argc, char ** argv){
    
    ros::init(argc, argv, "doctor");

    Doctor doctor = Doctor();

    

    return 0;
}