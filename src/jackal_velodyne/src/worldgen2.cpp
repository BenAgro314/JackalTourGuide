#include "xml_writer2.h"

using namespace std;


/*
 * create a function called create_subgraph:
 * 
 * Inputs: 
 * - the number of tables 
 * - the number of rows and cols 
 * - the start x and y pos
 * - the height and width of a waypoint 
 * - the off limit points 
 * - the point where no furnature can go
 * - table and chair model (optional)
 * 
 * Outputs:
 * - a subgraph filled with correct and connected waypoints 
 * - a list of acceptable id's for people to walk
 * - a list of correct tables
 * 
 */
 
struct Section{
	
	int rows;
	int cols;
	double start_x;
	double start_y;
	double width;
	double height;
	int size;
	set <vector<int>> off_lim;
	set <vector<int>> false_furn;
	string table_model;
	string chair_model;
	
	Section(int r, int c, double sx, double sy, double w, double h, set<vector<int>> off, set<vector<int>> furn, string t_model = "table_conference_2", string c_model = "chair_3"){
		rows = r;
		cols = c;
		size = rows*cols;
		start_x = sx;
		start_y = sy;
		width = w;
		height = h;
		off_lim = off;
		false_furn = furn;
		table_model = t_model;
		chair_model = c_model;
	}
	
};
 
struct Info{
	
	Graph sub_graph;
	vector<int> acc_ids;
	vector<Table> tables;
	
	Info(Graph G, vector<int> acc, vector<Table> t){
		sub_graph = G;
		acc_ids = acc;
		tables = t;
	}
	
};

Info create_subgraph(int n_tables, Section S){//int rows, int cols, double start_x, double start_y, double width, double height, set <vector<int>> off_lim, set <vector<int>> false_furn, string table_model = "table_conference_2", string chair_model = "chair_3")
	
	Graph G = Graph();
	vector<int> acc_ids;
	vector <Table> tables;
	
	vector <vector <WayPoint *>> global_points; // a grid of all waypoints
	vector <WayPoint *> furb_points; // a list of waypoints suitable for furnature
	
	populate_points(S.start_x, S.start_y, S.rows,S.cols, S.off_lim, S.false_furn, global_points, G, furb_points,S.width,S.height); //adds waypoints to graph and populates global_points and furb_points. 
	tables = add_tables(n_tables,furb_points, S.table_model, S.chair_model);
	add_edges(S.rows,S.cols, global_points , G, acc_ids); // connecting waypoints in graph and filling acc_ids
	
	Info res = Info(G, acc_ids, tables);
	return res;
}

int main(int argc, char ** argv){
	

	int num_people;
	int num_tables;
	
	ros::init(argc, argv, "worldgen");
	ros::NodeHandle nh("~");
	
	if (!nh.getParam("num_people", num_people)){
		num_people = 7;
	}
	
	if (!nh.getParam("num_tables", num_tables)){
		num_tables = 4;
	
	}

	srand( (unsigned)time(NULL));
	
	
	//SPECIFICATION OF SECTION 1
	
	Section S1 = Section(
	9,5,
	-6,8,
	3,3, 
	{{1,1},{1,2},{1,3},{4,1},{4,3},{5,1},{5,3},{5,4},{6,3},{6,4},{7,3},{7,4},{8,3},{8,4}}, 
	{{0,0},{0,1},{0,2},{0,3},{0,4},{1,0},{1,4},{2,0},{3,0},{4,0},{4,4},{5,0},{5,2},{6,0},{6,2},{2,2},{3,2}});
	
	
	//SPECIFICATION OF SECTION 2

	Section S2 = Section(
	2,5,
	-6,19, 
	3, 5,
	{{1,0}},
	{{1,4}});
	
	//SPECIFICATION OF SECTION 3
	
	Section S3 = Section(
	1, 3,
	-15,7.7,
	3,3,
	{},
	{{0,0},{0,1},{0,2}}
	);
	
	double total_size =(double) S1.size + S2.size;// + S3.size;
	
	
	///for now, we will simply divy up the tables by the number of waypoints in each section, regardless as to whether or not they are off limits
	int n_tables1 = (int) (num_tables * (double)((double) S1.size/(total_size))); // evenly distribute tables between sections 
	int n_tables2 = num_tables - n_tables1;// (int) (num_tables * (double)((double) S2.size/(total_size))); // evenly distribute tables between sections 
	//int n_tables3 = num_tables - n_tables1 - n_tables2;
	
	Info I1 = create_subgraph(n_tables1, S1);
	Info I2 = create_subgraph(n_tables2, S2);
	//Info I3 = create_subgraph(n_tables3, S3);
	
	vector<vector<int>> new_edges1;
	vector<vector<int>> new_edges2 = {{0,2}};
	Graph sum = add_graphs(I1.sub_graph, I2.sub_graph, new_edges1); 

	//Graph sum = add_graphs(s1,I3.sub_graph, new_edges2);
	vector <int> acc_ids;
	
	for (int id: I1.acc_ids){
		acc_ids.push_back(id);
	}
	for (int i =0; i<(int) I2.acc_ids.size(); ++i){ // we must appropriately adjust the acceptable id's 
		acc_ids.push_back(I2.acc_ids[i]+(int)I1.sub_graph.adj_list.size());
	}
	/*
	for (int i =0; i<(int) I3.acc_ids.size(); ++i){ // we must appropriately adjust the acceptable id's 
		acc_ids.push_back(I3.acc_ids[i]+(int)I1.sub_graph.adj_list.size()+(int)I2.sub_graph.adj_list.size());
	}
	*/
	
	vector <Person> people = fill_routes(num_people, sum, acc_ids); //we need a properyl connected graph and the acceptable id's
	
	vector <vector<Table>> tables_list = {I1.tables,I2.tables};//, I3.tables};
	vector <vector<Person>> people_list = {people};
	
	//WRITE TO FILE
	
	ofstream out;
	out.open("./src/jackal_velodyne/worlds/out2.world");
	ifstream in("./src/jackal_velodyne/src/include/template2.txt"); //use empty_template.txt to remove myhal, template2.txt otherwise
	
	if (in){
		cout << "Template Found" << endl;
	} else{
		cout << "Template Not Found" << endl;
	}
	
	
	write_launch(in, out, tables_list, people_list);

	ROS_WARN("\n\nMYHAL WORLD GENERATED WITH %d PEOPLE\n\n", num_people);
	ROS_WARN("\n\nMYHAL WORLD GENERATED WITH %d TABLES\n\n", num_tables);
	
	in.close();
	out.close();

	
	for (int i =0; i<sum.adj_list.size(); i++){
		delete sum.points[i];
	}

	return 0;
	
	
}
