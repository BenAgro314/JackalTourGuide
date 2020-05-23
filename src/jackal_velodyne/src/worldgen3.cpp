#include "worldgen3.h"
#define DEBUG 1

using namespace std;





int main(int argc, char ** argv){


	srand( (unsigned)time(NULL));
	int num_people;
	int num_tables;
	
	ros::init(argc, argv, "worldgen3");
	ros::NodeHandle nh;
	
	if (!nh.getParam("num_people", num_people)){
		num_people = 5;
	}
	
	if (!nh.getParam("num_tables", num_tables)){
		num_tables = 5;
	
	}
	
	
	// MAIN ATRIUM 

	vector_list table_points1 = vector_list({{-3,2}, {3,2},{-3,-3},{3,-3}});
	vector <ignition::math::Vector2d> polygon1;
	
	add_rectangle(-8.5,-5,8,8.5,polygon1);
	add_rectangle(-3.75,3.7,5.15,6.8,polygon1); //block off staircase
	
	Section atrium = Section(polygon1, ((8+8.5)*(8.5+5.5))-((5.15+3.17)*(6.8-3.7)), table_points1);

	//Upper middle room
	vector_list table_points2 = vector_list({{-3,12}, {3,12}, {-3,16}, {3,16}});
	vector <ignition::math::Vector2d> polygon2;
	add_rectangle(-8,10,8,20,polygon2);
	Section upper_middle = Section(polygon2, (10*16), table_points2, "table_conference_1", "chair_1");
	
	//Lower arium
	
	vector_list table_points3 = vector_list({{-6,-11}, {-1,-11}, {-6,-17}, {-1,-17}});
	vector <ignition::math::Vector2d> polygon3;
	add_rectangle(-8,-20,2,-9,polygon3);
	Section lower_atrium = Section(polygon3, (10*11), table_points3,"table_conference_3", "chair_2");
	
	// lower horizontal hallway
	
	vector <ignition::math::Vector2d> polygon4;
	add_rectangle(-14,-7.5,14,-6.5,polygon4);
	Section lower_h_hallway = Section(polygon4, (1*28), vector_list());
	
	
	// upper horizontal hallway
	
	vector <ignition::math::Vector2d> polygon5;
	add_rectangle(-14,7,14,8,polygon5);
	Section upper_h_hallway = Section(polygon5, (1*28), vector_list());
	
	
	//Left study space 
	
	vector <ignition::math::Vector2d> polygon6;
	add_rectangle(-13,-5,-10,5,polygon6);
	Section l_study_space = Section(polygon6, (3*10), vector_list({{-11.5,0.5}, {-11.5,3.5}, {-11.5,-2.5}}),"desk_yellow");
	
	vector <Section> sections;
	// add sections to list
	#if DEBUG == 1
	
	sections = {atrium, upper_middle, lower_atrium, l_study_space, lower_h_hallway, upper_h_hallway};
	
	#elif DEBUG == 2
	
	 //test retangle
	vector <ignition::math::Vector2d> test_poly;
	add_rectangle(-10,-10,10,10,test_poly);
	sections = {Section(test_poly, 20*20, vector_list())};


	#endif

	double total_area = 0;
	int total_tables = 0;
	
	for (Section s: sections){
		total_area+=s.area;
		total_tables += (int) s.table_points.list.size();
	}
	
	int rem_people = num_people;
	int rem_tables = num_tables;
	
	for (int i = 0; i< (int) sections.size()-1; i++){
		Section s = sections[i];
		int sn_tables = (int) round(((double)num_tables)*((double)s.table_points.list.size()/(double)total_tables));
		rem_tables -= sn_tables;
		int sn_people = (int) round(((double)num_people)*(s.area/total_area));
		//printf("tables: %d, people: %d, i: %d\n", sn_tables, sn_people, i);
		rem_people -= sn_people;
		sections[i].gen_tables(sn_tables);
		sections[i].gen_people(sn_people);
	}
	
	sections.back().gen_people(rem_people);
	sections.back().gen_tables(rem_tables);
	//printf("tables: %d, people: %d\n", rem_tables, rem_people);
	
	ofstream out;
	out.open("./src/jackal_velodyne/worlds/worldgen3.world");
	ifstream in;

	#if DEBUG == 1
	in = ifstream("./src/jackal_velodyne/src/include/template2.txt"); //use empty_template.txt to remove myhal, template2.txt otherwise
	#elif DEBUG == 2
	in = ifstream("./src/jackal_velodyne/src/include/empty_template.txt"); //use empty_template.txt to remove myhal, template2.txt otherwise
	#endif

	if (in){
		cout << "Template Found" << endl;
	} else{
		cout << "Template Not Found" << endl;
	}
	
	write_file(in,out,sections);

	ROS_WARN("\n\nMYHAL WORLD GENERATED WITH %d PEOPLE\n\n", num_people);
	ROS_WARN("\n\nMYHAL WORLD GENERATED WITH %d TABLES\n\n", num_tables);
	
	in.close();
	out.close();
	
	return 0;
}
