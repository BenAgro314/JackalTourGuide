#include "post_classifier.hh"



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

    classifier.WriteToPLY();

    return 0;
}

Classifier::Classifier(std::string filename, std::string username){
    this->filename = filename;
    this->username = username;
    this->filepath = "/home/" + this->username + "/Myhal_Simulation/simulated_runs/" + this->filename + "/";
    
}

void Classifier::Load(){
    std::cout << "Classifying files in: " << this->filepath << std::endl;
    std::string command = "mkdir " + this->filepath + "classified_frames/";
    system(command.c_str());

    // read trajectory and frames

    DataProcessor processor = DataProcessor(this->filename, false);

    processor.SetTopics("/ground_truth/state", {"/velodyne_points"});

    FramesAndTraj data = processor.GetData();

    this->robot_trajectory = data.trajectory;
    this->input_frames = data.frames;

    // read static objects 


    // interpolate trajectory and offset frames 

    double min_traj_time;
    double max_traj_time;

    if (this->robot_trajectory.size() == 0){
        std::cout << "Invalid robot trajectory: size 0\n";
        return;
    }

    min_traj_time = this->robot_trajectory[0].time;
    max_traj_time = this->robot_trajectory[this->robot_trajectory.size()-1].time;


    int count = 0;
    for (auto frame: this->input_frames){
        double time = frame.Time();
        auto points = frame.Points();

        // find neighbouring pose measurments

        if (time < min_traj_time || time > max_traj_time){
            std::cout << "Dropping frame at time " << time << " because it is out of range of the trajectory times\n";
            continue;
        }

        // binary search

        int lower_ind = count;
        
        while (this->robot_trajectory[lower_ind].time > time || this->robot_trajectory[lower_ind+1].time < time){
            
            if (this->robot_trajectory[lower_ind].time > time){
                lower_ind = std::floor(lower_ind/2);
            } else {
                lower_ind = lower_ind + std::floor(((this->robot_trajectory.size()-1)-lower_ind)/2);
            }
        }
        
        //std::printf("lower_time: %f, upper_time: %f, time: %f\n", this->robot_trajectory[lower_ind].time, this->robot_trajectory[lower_ind+1].time, time);
        //std::printf("(%f, %f, %f) -> (%f, %f %f)\n", this->robot_trajectory[lower_ind].pose.Pos().X(), this->robot_trajectory[lower_ind].pose.Pos().Y(), this->robot_trajectory[lower_ind].pose.Pos().Z(), this->robot_trajectory[lower_ind+1].pose.Pos().X(), this->robot_trajectory[lower_ind+1].pose.Pos().Y(), this->robot_trajectory[lower_ind+1].pose.Pos().Z());
        auto pose1 = this->robot_trajectory[lower_ind].pose;
        
        auto pose2 = this->robot_trajectory[lower_ind+1].pose;


        //watch for the switch between w and x 
        auto trans = ignition::math::Pose3d(0,0,0.539,1,0,0,0) + ignition::math::Pose3d(0,0,0,1,0,0,0) + ignition::math::Pose3d(0,0,0.0377,1,0,0,0);

        pose1+=trans;
        pose2+=trans;
        double t1 = this->robot_trajectory[lower_ind].time;
        double t2 = this->robot_trajectory[lower_ind+1].time;

        //std::printf("(%f, %f, %f, %f, %f, %f, %f)\n", pose1.Pos().X(), pose1.Pos().Y(), pose1.Pos().Z(), pose1.Rot().X(), pose1.Rot().Y(), pose1.Rot().Z(), pose1.Rot().W());

        auto test_pos = ((pose2.Pos() - pose1.Pos())/(t2-t1))*(time-t1)+pose1.Pos();
        ignition::math::Pose3d translation;
        translation.Pos() = test_pos;

        double a = (time - t1)/(t2 - t1);

        auto rotation = ignition::math::Quaterniond::Slerp(a,pose1.Rot(),pose2.Rot());


        Frame translated_frame = Frame(false); // no pose specified
        translated_frame.SetTime(time);
        bool first =true;
        for (auto point: frame.Points()){

            auto translated_pt = rotation.RotateVector(ignition::math::Vector3d(point.X(), point.Y(), point.Z()));
            translated_pt = translation.CoordPositionAdd(translated_pt);

            translated_frame.AddPoint(translated_pt);
        }

        this->output_frames.push_back(translated_frame);
        count++;
        
    }


    // iterate through the points of each frame, check collisions against static objects, classify points 



    std::cout << "Translated and classified points\n";
}

void Classifier::WriteToPLY(){
    std::cout << "Writing data to .ply files\n";
    for (auto frame: this->output_frames){
        frame.WriteToFile(this->filepath + "classified_frames/");
    }
}