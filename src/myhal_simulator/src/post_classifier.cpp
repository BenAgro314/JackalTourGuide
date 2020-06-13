#include "post_classifier.hh"

#define TRANSLATE true

int main(int argc, char ** argv){

    if (argc ==1){
        std::cout << "must input bag name\n";
        return 0;
    }

    std::string user_name = "default";
    if (const char * user = std::getenv("USER")){
      user_name = user;
    } 

    bool colorize = false;
    if (argc > 2){
        colorize = true;
        std::cout << "colorizing points\n";
    }

    std::string time_name = argv[1];

    Classifier classifier = Classifier(time_name, user_name, colorize);

    classifier.Load();

    classifier.WriteToPLY();

    return 0;
}

Classifier::Classifier(std::string filename, std::string username, bool colorize){
    this->filename = filename;
    this->username = username;
    this->filepath = "/home/" + this->username + "/Myhal_Simulation/simulated_runs/" + this->filename + "/";
    this->colorize = colorize;
    
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

    std::cout << "Reading static objects" << std::endl;

    happly::PLYData plyIn(this->filepath + "static_objects.ply");

    this->static_objects = ReadObjects(plyIn);

    double min_x = 10e9;
    double min_y = 10e9;
    double max_x = -10e9;
    double max_y = -10e9;

    for (auto obj: static_objects){
        min_x = std::min(obj.MinX(), min_x);
        min_y = std::min(obj.MinY(), min_y);
        max_x = std::max(obj.MaxX(), max_x);
        max_y = std::max(obj.MaxY(), max_y);
    }

    auto qt_box = ignition::math::Box(min_x-1, min_y-1, 0, max_x+1, max_y+1, 0);
    //std::printf("(%f, %f) -> (%f, %f)\n", min_x, min_y, max_x, max_y);
    this->static_quadtree = boost::make_shared<QuadTree>(qt_box);

    for (auto obj: static_objects){
        auto new_ptr = boost::make_shared<BoxObject>(obj);
        auto box = obj.Box();
        box.Min().Z() = 0;
        box.Max().Z() = 0;
        auto new_node = QTData(box, new_ptr, box_type);
        this->static_quadtree->Insert(new_node);
    }

    // auto t_box = ignition::math::Box(ignition::math::Vector3d(-10,-10,0), ignition::math::Vector3d(10,-9,0));
    // auto new_ptr = boost::make_shared<BoxObject>(t_box, 2);
    // auto new_node = QTData(t_box, new_ptr, box_type);
    // this->static_quadtree->Insert(new_node);

    // interpolate trajectory and offset frames 

    double min_traj_time;
    double max_traj_time;

    if (this->robot_trajectory.size() == 0){
        std::cout << "Invalid robot trajectory: size 0\n";
        return;
    }

    min_traj_time = this->robot_trajectory[0].time;
    max_traj_time = this->robot_trajectory[this->robot_trajectory.size()-1].time;


    int last = 0;

    std::cout << "Classifying Frames\n";

    

    for (auto frame: this->input_frames){
        double time = frame.Time();
        auto points = frame.Points();

        // find neighbouring pose measurments

        if (time < min_traj_time || time > max_traj_time){
            std::cout << "Dropping frame at time " << time << " because it is out of range of the trajectory times\n";
            continue;
        }

        
        int lower_ind = last;
        
        while (this->robot_trajectory[lower_ind].time > time || this->robot_trajectory[lower_ind+1].time < time){
            
            if (this->robot_trajectory[lower_ind].time > time){
                lower_ind -=1;
            } else {
                lower_ind +=1;
            }
        }
        
        last = lower_ind+1;
      
        auto pose1 = this->robot_trajectory[lower_ind].pose;
        
        auto pose2 = this->robot_trajectory[lower_ind+1].pose;

       
        auto tf_trans = ignition::math::Pose3d(0,0,0.539,1,0,0,0) + ignition::math::Pose3d(0,0,0,1,0,0,0) + ignition::math::Pose3d(0,0,0.0377,1,0,0,0);

        pose1+=tf_trans;
        pose2+=tf_trans;
        double t1 = this->robot_trajectory[lower_ind].time;
        double t2 = this->robot_trajectory[lower_ind+1].time;

    
        auto translation = utilities::InterpolatePose(time, t1, t2, pose1, pose2);

        Frame translated_frame = Frame(false); // no pose specified
        translated_frame.SetTime(time);
        bool first =true;

        ignition::math::Vector3d min_p;
        for (auto point: frame.Points()){
            
            auto trans_pt = translation.CoordPositionAdd(ignition::math::Vector3d(point.X(), point.Y(), point.Z()));

       
            

            //check point collisions;

            int cat =0;

            if (trans_pt.Z() <= 0){
                cat = 0; // ground
                
                #if TRANSLATE
                    translated_frame.AddPoint(trans_pt, cat);
                #else
                    translate_frame.AddPoint(ignition::math::Vector3d(point.X(), point.Y(), point.Z()), cat);
                #endif
                continue;
            }

            std::vector<boost::shared_ptr<BoxObject>> near_objects;

            double resolution = 0.05;
            auto min = ignition::math::Vector3d(trans_pt.X() - resolution, trans_pt.Y() - resolution, 0);
            auto max = ignition::math::Vector3d(trans_pt.X() + resolution, trans_pt.Y() + resolution, 0);
            auto query_range = ignition::math::Box(min,max);
            
            std::vector<QTData> query_objects = this->static_quadtree->QueryRange(query_range);
            for (auto n: query_objects){
                if (n.type == box_type){
                    near_objects.push_back(boost::static_pointer_cast<BoxObject>(n.data));
                    
                }
            }

            if (near_objects.size() == 0){
                cat = 2; //moving_actor
                
                #if TRANSLATE
                    translated_frame.AddPoint(trans_pt, cat);
                #else
                    translate_frame.AddPoint(ignition::math::Vector3d(point.X(), point.Y(), point.Z()), cat);
                #endif
                continue;
            }

            double min_dist = trans_pt.Z();

            for (auto n: near_objects){
                
                auto dist = utilities::dist_to_box(trans_pt, n->Box());
                if (n->Cat() == 3){
                    cat = n->Cat();
                    break;
                }
                if (dist <= min_dist){
                    min_dist = dist;
                    cat = n->Cat();
                }
            }
            
            
            //std::cout << cat << std::endl;
            
            #if TRANSLATE
                translated_frame.AddPoint(trans_pt, cat);
            #else
                translate_frame.AddPoint(ignition::math::Vector3d(point.X(), point.Y(), point.Z()), cat);
            #endif
            
        }

        this->output_frames.push_back(translated_frame);
        
    }

    std::cout << "Translated and classified points\n";
}

void Classifier::WriteToPLY(){
    std::cout << "Writing data to .ply files\n";
    for (auto frame: this->output_frames){
        frame.WriteToFile(this->filepath + "classified_frames/", this->colorize);
    }
}