#include "graphs.h"

using namespace std;


struct Table{
	WayPoint *pos;
	bool vert;
	int chairs;
	int loit; //number of people standing around the table 
	string table_model;
	string chair_model;
	Table(WayPoint  *p, bool v = false, int c =0 , int l = 0, string t_model = "table_conference_2", string c_model = "chair_3"){
		pos = p;
		vert = v;
		chairs = c;
		loit = l;
		table_model = t_model;
		chair_model = c_model;
	}
};

struct Person{
	
	string id;
	vector <WayPoint> goals;
	double speed;
	
	Person(string i, double s){
		id = i;
		speed = s;
	}
	
	void add_goal(WayPoint point){
		goals.push_back(point);
	}
	
};


void write_waypoint(double x, double y, double angle, double time, ofstream& out){
	
	out << "\t<waypoint>" << endl;
	out << "\t\t<time> " << time << "</time>" << endl;
	out << "\t\t<pose> " << x << " " << y << " " << 0 << " " << 0 << " " << 0 << " " << angle << " </pose>" << endl;
	out << "\t</waypoint>" << endl;
}



vector<Table> add_tables(int num, vector <WayPoint *> &furb_points, string table_model, string chair_model){
	
	vector<Table> tables;
	
	
	random_shuffle(furb_points.begin(), furb_points.end());
	int count = 0;
	for (int i =0; i<min(num,(int) furb_points.size()); i++){
		WayPoint * w = furb_points[i];
		bool vert = ((w->row)%2)^((w->col)%2);
		tables.push_back(Table(w, vert, rand()%4 + 1, rand()%2, table_model, chair_model)); //come back to randomize orientation, chair number, (come back to randomize num people)
		furb_points[i]->filled = true;
		count++;
	}
	//cout << "added " << count << " tables" << endl;
	return tables;
}


int twoD_to_ind(int row, int col, int width){
	return row*width + col;
}

// add WayPoints and nodes
void populate_points(int start_x, int start_y, int rows, int cols, set<vector<int>> occ_points, set <vector<int>> false_furb, vector <vector <WayPoint *>> &global_points, Graph &G, vector <WayPoint *> &furb_points, double width = 3, double height = 3){
	
	// populate global_points grid
	
	
	for (int row =0; row < rows; row++){
		for (int col =0; col<cols; col++){
			global_points.push_back({});
			
			
			double x = start_x+width*col;
			double y = start_y-height*row;
			
			global_points[row].push_back(new WayPoint(x,y));
			global_points[row][col]->row = row;
			global_points[row][col]->col = col;
			G.add_node(global_points[row][col]);
			
			if (occ_points.find({row,col}) != occ_points.end()){ //if the node is occupied by walls 
				global_points[row][col]->filled = true;
				global_points[row][col]->furb = false;
			} 
			else if (false_furb.find({row,col}) != false_furb.end()){ //if the node is not suitable for furnature 
				global_points[row][col]->furb = false;
			}
			else{
				furb_points.push_back(global_points[row][col]);
			}
			
			
		}
	}

}

void add_edges(int rows, int cols, vector <vector <WayPoint *>> &global_points, Graph &G, vector <int> &acc_ids){
	for (int row =0; row < rows; row++){
		for (int col =0; col<cols; col++){
			if (!global_points[row][col]->filled){
				acc_ids.push_back(twoD_to_ind(row,col,cols));
				bool left = col>0;
				bool right = col<cols-1;
				bool top = row> 0;
				bool bot = row < rows-1;
				
				if (left){
					
					if (!global_points[row][col-1]->filled) add_by_id(G, twoD_to_ind(row,col,cols), twoD_to_ind(row,col-1, cols));
				}
				if (right){
					if (!global_points[row][col+1]->filled) add_by_id(G, twoD_to_ind(row,col,cols), twoD_to_ind(row,col+1, cols));
				}
				if (top){
					if (!global_points[row-1][col]->filled) add_by_id(G, twoD_to_ind(row,col,cols), twoD_to_ind(row-1,col, cols));
				}
				if (bot){
					if (!global_points[row+1][col]->filled) add_by_id(G, twoD_to_ind(row,col,cols), twoD_to_ind(row+1,col, cols));
				}
			}
		}
	}
}

void path_finder(Graph G, WayPoint * A, WayPoint * B, vector<WayPoint> &path){ //simple depth first search, will change this
	

	vector <WayPoint *> stack;
	stack.push_back(A);
	map <int, WayPoint *> prev;
	set <int> visited;
	
	bool found = false;
	WayPoint * end = A;
	
	while (stack.size() > 0 && (!found)){
		WayPoint * n = stack.back();
		stack.pop_back();
		if (visited.find(n->id) != visited.end()){
			continue;
		}
		if (n == B){
			found = true;
		}
		
		int row = n->id;
		end = n;
		
		vector <WayPoint *> list = G.adj_list[row];
		random_shuffle(list.begin(), list.end());
		for (WayPoint * curr: list){
			if (visited.find(curr->id) == visited.end()){
				prev[curr->id] = n;
				stack.push_back(curr);
				
			}
			
		}
		
		visited.insert(n->id);
	}
	
	if (found){
		end = B;
	}
	
	path.push_back(*end);
	WayPoint * curr = end;
	while (curr->id != A->id){
		WayPoint * parent = prev[curr->id];
		WayPoint val = *(parent);
		//randomness if desired
		//val.x += ((rand()%1)/2)-0.25;
		//val.y += ((rand()%1)/2)-0.25;
		path.push_back(val);
		curr = parent;
	}
	
	if (path.size() > 2){
		unsigned int i = path.size()-2;
		while (i > 0){
			path.push_back(path[i]);
			i--;
		}
	}
}


void path_by_id(Graph G, int a, int b, vector<WayPoint> &path){
	WayPoint * A = G.points[a];
	WayPoint * B = G.points[b];
	path_finder(G,A,B,path);
}

vector <double> shift(WayPoint A, WayPoint B){
	
	double angle = angle_finder(A,B);
	
	vector <double> res; //res[0] is x shift, res[1] is y shift
	
	if (abs(angle-1.57) < 0.1){ // if we are angled upwards
		res = {0.5,0};
	} else if (abs(angle+1.57) < 0.1){ //down
		res = {-0.5,0};
	} else if (abs(angle) < 0.1){ // left
		res = {0,-0.5};
	} else if (abs(abs(angle)-3.141) < 0.1){ //right 
		res = {0,0.5};
	} else {
		res = {0,0};
	}
	
	
	return res;
}


void write_point(double x, double y, double angle, ofstream& out, bool target = true){
	if (!target){
		out << "\t<pose>\n";
	} else{
		out << "\t<target>\n";
	}
	
	out << "\t\t" << x << "\n" << "\t\t" << y << "\n" << "\t\t" << 1 << "\n" << "\t\t" << angle << "\n" << "\t\t" << 0 << "\n" << "\t\t" << 3.1415 << "\n";
	
	
	if (!target){
		out << "\t</pose>\n\n";
	} else{
		out << "\t</target>\n\n";
	}
}

void write_wanderer(double min_x, double max_x, double min_y, double max_y, WayPoint start, ofstream& out, double speed = 0.7){
	
	int col_num = rand()%3;
	string color;
	
	if (col_num == 0){
		color = "red";
	} else if (col_num == 1){
		color = "blue";
	} else {
		color= "green";
	}
	
	out << "\n";
	out << "<actor name=\"human_" << min_x << start.x << start.y << rand()%10000 <<"\">\n";
	
	write_point(start.x, start.y, 0, out, false);
	
	out << "\n\t<skin>\n\t<filename>model://actor/meshes/SKIN_man_" << color << "_shirt.dae</filename>\n\t</skin>\n";
	
	out << "\t<animation name=\"animation\"> \n\t\t<filename>model://actor/meshes/ANIMATION_walking.dae</filename>\n\t\t<interpolate_x>true</interpolate_x>\n\t</animation>\n";
    
    
    // write the bounds and starting point 
    
    out << "\t\n<plugin name=\"trajectory\" filename=\"libTrajectoryActorPlugin.so\">\n";
    
    out << "<random>true</random>\n";
    
    out << "<boundary>\n";
    out << min_x << endl;
    out << max_x << endl;
    out << min_y << endl;
    out << max_y << endl;
    out << "</boundary>\n";
    
    write_point(start.x, start.y, 0, out, false);
    
    
    out << "\n<velocity>" << speed << "</velocity>\n";
	out << "<obstacle_margin>0.5</obstacle_margin>\n";
	
    
    out << "</plugin>\n";
    
    
    
    out << "</actor>\n";
	
}


void write_actor(Person actor, ofstream& out, bool shifted = true){
	
	if (actor.goals.size() == 1){
		WayPoint point = actor.goals[0];
		write_wanderer(point.x-1.5, point.x+1.5, point.y-1.5, point.y+1.5, point, out, actor.speed/2);
		return;
	}
	
	// randomize shirt color
	
	int col_num = rand()%3;
	string color;
	
	if (col_num == 0){
		color = "red";
	} else if (col_num == 1){
		color = "blue";
	} else {
		color= "green";
	}
	
	WayPoint prev = actor.goals[actor.goals.size()-1];
	WayPoint start = actor.goals[0];
	WayPoint second = actor.goals[1%actor.goals.size()];
	
	vector <double> curr_shift = shift(start,prev);
	vector <double> next_shift = shift(start,second);
	
	vector <double> past_shift = {0,0};
	
	if (abs(abs(angle_finder(prev,start) - angle_finder(start,second))-1.57) < 0.1){ //if change axis
		past_shift[0] = next_shift[0]+curr_shift[0];
		past_shift[1] = next_shift[1]+curr_shift[1];
	} else { // if we stay on the same axis
		past_shift[0] = next_shift[0];
		past_shift[1] = next_shift[1];
	}
		 
	out << "\n";
	out << "<actor name=\"human_" << actor.id << actor.goals[0].x << rand()%10000 <<"\">\n";
	
	// write the starting position 
	
	int length = (int) actor.goals.size();
	
	
	if (shifted){
		write_point(start.x + past_shift[0], start.y + past_shift[1], angle_finder(start,second), out, false); // writing the waypoint positioning for A->B
	} else{
		write_point(start.x, start.y, angle_finder(start,second), out, false);
	}
	
	// write the skin
	
	out << "\n\t<skin>\n\t<filename>model://actor/meshes/SKIN_man_" << color << "_shirt.dae</filename>\n\t</skin>\n";
	//out << "\n\t<skin>\n\t<filename>model://elegent_male/meshes/elegent_male.dae</filename>\n\t</skin>\n";
	// write the animation 
	
	out << "\t<animation name=\"animation\"> \n\t\t<filename>model://actor/meshes/ANIMATION_walking.dae</filename>\n\t\t<interpolate_x>true</interpolate_x>\n\t</animation>\n";
    
    
    // write the trajectory 
    
    out << "\t\n<plugin name=\"trajectory\" filename=\"libTrajectoryActorPlugin.so\">\n";
    
  
	
    
    for (int i =0; i< length; i++){
/* 
		 * If we are currently moving 
		 * up: shift +0.5 in x (right)
		 * down: shift -0.5 in x (left)
		 * left: shift +0.5 in y (up)
		 * right: shift -0.5 in y (down);
		 * 
		 * 
		 * we must also consider the shift to the next point due to the next direction
		 * 
		 * if we are going to be moving 
		 * up: shift +0.5 in x (right)
		 * down: shift -0.5 in x (left)
		 * left: shift +0.5 in y (up)
		 * right: shift -0.5 in y (down);
		 */
		
		WayPoint A = actor.goals[i]; // the point we are currently at 
		WayPoint B = actor.goals[(i+1)%actor.goals.size()]; // the point we are going to
		WayPoint C = actor.goals[(i+2)%actor.goals.size()]; // the following point 
		
		
		curr_shift = shift(A,B);
		next_shift = shift(B,C);
		
		// determine if we are changing dir on the same axis:
		
		if (shifted){
			write_point(A.x + past_shift[0], A.y + past_shift[1], angle_finder(A,B), out); // writing the waypoint positioning for A->B
		} else{
			write_point(A.x, A.y, angle_finder(A,B), out);
		}
		
		double x_shift = 0;
		double y_shift = 0;
		
		if ((curr_shift[0] != 0 && next_shift[1] != 0) || (curr_shift[1] != 0 && next_shift[0] != 0)){ //if change axis
			
			x_shift = next_shift[0]+curr_shift[0];
			y_shift = next_shift[1]+curr_shift[1];
			past_shift = {x_shift, y_shift}; // setting past shift so it is the same after the reposition rotation
		} else { // if we stay on the same axis
			x_shift = curr_shift[0];
			y_shift = curr_shift[1];
			past_shift = {next_shift[0], next_shift[1]};
		}
		
		if (shifted){
			write_point(B.x + x_shift, B.y + y_shift, angle_finder(A,B), out); // our next destination (maintaining current positioning)
		} else{
			write_point(B.x, B.y, angle_finder(A,B), out); // our next destination (maintaining current positioning)
		}
	}
	
	// write trajectory characteristics
	
	out << "\n<velocity>" << actor.speed << "</velocity>\n";
	out << "<obstacle_margin>0.5</obstacle_margin>\n";
	
    out << "<obstacle>jackal</obstacle>\n";
    
    out << "</plugin>\n";
    
    //out << "<plugin name=\"attach_model\" filename=\"libAttachModelPlugin.so\">\n<link>\n<link_name>human_" << actor.id <<"_pose</link_name>\n<model>\n<model_name>human_" << actor.id <<"_collision_model</model_name>\n</model>\n</link>\n</plugin>\n";
    
    out << "</actor>\n";

}


void write_stander(WayPoint point, double angle, ofstream &out, bool walking = false, WayPoint dest = WayPoint(0,0)){
	
	 string f_name = "stand.dae";
	 string a_name = "stand.dae";
	
	if (!walking){
		out << "\n<actor name=\""<< point.x << point.y << angle << "\"> \n <skin> \n <filename>" << f_name << "</filename> \n </skin> \n <animation name=\"walking\"> \n <filename>" << a_name << "</filename> \n </animation> \n";
		out << "<script> \n<trajectory id=\"0\" type=\"walking\">";
		
		write_waypoint(point.x, point.y, angle, 0, out);
		write_waypoint(point.x, point.y, angle, 10, out);    
		
		out << "\n</trajectory>\n</script>\n</actor>\n";
	} else {
		string name = to_string(point.x+point.y+angle+(rand()%100));
		double s = (double (rand()%50))+50;
		Person P = Person(name, s/100);
		P.add_goal(point);
		P.add_goal(dest);
		write_actor(P, out, false);
	}
}


void write_chair(double x, double y, double angle, ofstream& out, string chair_model){
	
		out << endl;
		
		out << "<include>\n";
		out << "<uri>model://"<< chair_model << "</uri> \n";
		out << "<name> \"" << x << y << rand()%100 << "\" </name> \n";

		//cout << angle_shift << endl;
		out << "<pose> " << x << " " << y << " " << 0 << " " << 0 << " " << 0 << " " << angle  << " </pose>\n";
		
		out << "</include>\n";
	
		out << endl;
}

void write_table(Table table, ofstream& out){
	
	string table_model = table.table_model;
	string chair_model = table.chair_model;
	
	out << endl;
	out << "<include>\n";
    out << "<uri>model://" << table_model << "</uri> \n";
    out << "<name> \"" << table.pos->x << table.pos->y << "\" </name> \n";
    out << "<pose> " << table.pos->x << " " << table.pos->y << " " << 0 << " " << 0 << " " << 0 << " " << 1.57*((int) table.vert) << " </pose>\n";
    out << "</include>\n";
    
    for (int i = 0; i< table.chairs; i++){
		
		/*
		 * Note:
		 * if (i&1) == 1, then we are on the left of the table
		 * if (i&2) == 2, then we are on the bottom of the table
		 * 
		 */
		 
		double x_shift = 0.5*((double) rand() / (RAND_MAX))+0.5; //0.5 to 1 meter way from the center of the table 
		double y_shift = 0.5*((double) rand() / (RAND_MAX))+0.5; //0.5 to 1 meter way from the center of the table 
		
		
		if (table.vert){
			double angle_shift = (x_shift)*3.1415*0.5*(-1+2*((double) rand() / (RAND_MAX))); // we must scale the allowable rotation shift by the distance from the table 
			write_chair(table.pos->x + x_shift-2*x_shift*(i&1), table.pos->y + y_shift-2*y_shift*((i&2) == 2), angle_shift + (((int) table.vert) == 0)*((i&2)==2)*3.14 + (((int) table.vert))*((i&1)*3.14-1.57), out, chair_model);
		} else{
			double angle_shift = (y_shift)*3.1415*0.5*(-1+2*((double) rand() / (RAND_MAX))); // we must scale the allowable rotation shift by the distance from the table 
			write_chair(table.pos->x + + x_shift-2*x_shift*(i&1), table.pos->y + y_shift-2*y_shift*((i&2) == 2), angle_shift + (((int) table.vert) == 0)*((i&2)==2)*3.14 + (((int) table.vert))*((i&1)*3.14-1.57), out, chair_model);
		}
		

	}
	
	for (int i =0; i<table.loit; i++){
		if (table.vert){
			if (i&1){//we are on the left
				write_wanderer(table.pos->x-2, table.pos->x-1, table.pos->y-1.5, table.pos->y+1.5, WayPoint(table.pos->x-1.5, table.pos->y), out, 0.6);
			} else{
				write_wanderer(table.pos->x+1, table.pos->x+2, table.pos->y-1.5, table.pos->y+1.5, WayPoint(table.pos->x+1.5, table.pos->y), out, 0.6);
			}
			
		} else{
			if ((i&2) == 2){//we are on the bottom
				write_wanderer(table.pos->x-1.5, table.pos->x+1.5, table.pos->y-2, table.pos->y-1, WayPoint(table.pos->x, table.pos->y-1.5), out, 0.6);
			} else{
				write_wanderer(table.pos->x-1.5, table.pos->x+1.5, table.pos->y+1, table.pos->y+2, WayPoint(table.pos->x, table.pos->y+1.5), out, 0.6);
			}
		}
		
	}
    
}


vector<Person> fill_routes(int num, Graph G, vector <int> acc_ids){
	vector <Person> people;
	random_shuffle(acc_ids.begin(), acc_ids.end());
	
	int num_add =0;

	for (int i =0; i< min(num, (int) acc_ids.size()-1); i++){
		
		//cout << num_add << endl;
		
		int start = acc_ids[i];
		
		int end = acc_ids[i+1];
		
		//cout << "start: " << start << " end: " << end << endl;
		
		double speed = 0.9+((((double) (rand()%100))/100)/2);
		Person p = Person(to_string(i), speed);
		path_by_id(G,end,start,p.goals);
		people.push_back(p);
		num_add ++;
	}
	//cout << "Added " <<  num_add << " people" << endl;
	return people;
}
	
void write_launch(ifstream& in, ofstream& out, vector <vector<Table>> tables, vector <vector<Person>> people){
	char str[255];
	int line =0;

	while(in) {
		in.getline(str, 255);  // delim defaults to '\n'
		if(in) {
			
			if (line == 105){
				// insert writing people and furnature here
				for (vector<Table> t_list: tables){
					for (Table t: t_list){
						write_table(t,out);
					}
				}
				
				for (vector <Person> p_list: people){
					for (Person p: p_list){
						write_actor(p, out);
					}
				}
			}

			out << str << endl;
			line++;

		}
	}	
}
