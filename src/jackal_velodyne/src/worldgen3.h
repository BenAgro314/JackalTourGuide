#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <set>
#include <stdlib.h>
#include <time.h>
#include <set>
#include <map>
#include <vector>
#include <random> 
#include <ignition/math/Line2.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Rand.hh>

#include <ros/ros.h>

///TODO: CHECK TOO MANY PEOPLE

using namespace std;

const double EPS = 1E-9;

struct pt {
    double x, y;
	
	pt(double _x, double _y){
		x = _x;
		y  =_y;
	}
    bool operator<(const pt& p) const
    {
        return x < p.x - EPS || (abs(x - p.x) < EPS && y < p.y - EPS);
    }
};

struct line {
    double a, b, c;

    line() {}
    line(pt p, pt q)
    {
        a = p.y - q.y;
        b = q.x - p.x;
        c = -a * p.x - b * p.y;
        norm();
    }

    void norm()
    {
        double z = sqrt(a * a + b * b);
        if (abs(z) > EPS)
            a /= z, b /= z, c /= z;
    }

    double dist(pt p) const { return a * p.x + b * p.y + c; }
};

double det(double a, double b, double c, double d)
{
    return a * d - b * c;
}

inline bool betw(double l, double r, double x)
{
    return min(l, r) <= x + EPS && x <= max(l, r) + EPS;
}

inline bool intersect_1d(double a, double b, double c, double d)
{
    if (a > b)
        swap(a, b);
    if (c > d)
        swap(c, d);
    return max(a, c) <= min(b, d) + EPS;
}

bool intersect(ignition::math::Line2d line1, ignition::math::Line2d line2, ignition::math::Vector2d &int1)
{	
	pt a = pt(line1[0].X(), line1[0].Y());
	pt b = pt(line1[1].X(), line1[1].Y());
	
	pt c = pt(line2[0].X(), line2[0].Y());
	pt d = pt(line2[1].X(), line2[1].Y());

    if (!intersect_1d(a.x, b.x, c.x, d.x) || !intersect_1d(a.y, b.y, c.y, d.y))
        return false;
    line m(a, b);
    line n(c, d);
    double zn = det(m.a, m.b, n.a, n.b);
    if (abs(zn) < EPS) {
        if (abs(m.dist(c)) > EPS || abs(n.dist(a)) > EPS)
            return false;
        if (b < a)
            swap(a, b);
        if (d < c)
            swap(c, d);
        pt left = max(a, c);
        pt right = min(b, d);
  
        int1.X() = left.x;
        int1.Y() = left.y;

        return true;
    } else {
		pt left = pt(-det(m.c, m.b, n.c, n.b) / zn, -det(m.a, m.c, n.a, n.c) / zn);
		pt right = pt(-det(m.c, m.b, n.c, n.b) / zn, -det(m.a, m.c, n.a, n.c) / zn);

        int1.X() = left.x;
        int1.Y() = left.y;

        return betw(a.x, b.x, left.x) && betw(a.y, b.y, left.y) && betw(c.x, d.x, left.x) && betw(c.y, d.y, left.y);
    }
}



struct Person{
	ignition::math::Vector2d start;
	vector<ignition::math::Vector2d> polygon;
	double speed;
	string plugin_name;
	Person(ignition::math::Vector2d _start, vector<ignition::math::Vector2d> _polygon, double _speed = 0.9, string _plugin_name = "libactor_plugin.so"){
		start = _start;
		polygon = _polygon;
		speed = _speed;
		plugin_name = _plugin_name;
	}
	
};

struct Table{
	ignition::math::Vector2d pos;
	bool vert;
	string table_model;
	string chair_model;
	
	int num_chairs;
	Table(ignition::math::Vector2d _pos, bool _vert, int _num_chairs = 0, string _table_model = "table_conference_2", string _chair_model = "chair_3"){
		vert = _vert;
		pos = _pos;
		table_model = _table_model;
		chair_model = _chair_model;
		num_chairs = _num_chairs;
		
	}
	
};


struct vector_list{
	vector <ignition::math::Vector2d> list;
	
	vector_list(vector<vector<double>> points = {}){
		for (vector<double> v: points){
			list.push_back(ignition::math::Vector2d(v[0],v[1]));
		}
		
	}
	
	void add_vector(double x, double y){
		list.push_back(ignition::math::Vector2d(x,y));
	}
	
};


// We will define an even length list of points to be a polygon where the for each even index i, (i,i+1) forms a line on the polygon (i = 0,2,4..)

void add_by_line(ignition::math::Line2d line, vector<ignition::math::Vector2d> &polygon){
	polygon.push_back(line[0]);
	polygon.push_back(line[1]);
}

void add_by_points(double x1, double y1, double x2, double y2, vector<ignition::math::Vector2d> &polygon){
	ignition::math::Vector2d p1 = ignition::math::Vector2d(x1,y1);
	ignition::math::Vector2d p2 = ignition::math::Vector2d(x2,y2);
	polygon.push_back(p1);
	polygon.push_back(p2);
}
/*
 * Write a function that randomly selects a point in a provided polygon and height bounds 
 * 
 * 1. choose a height level, and extend a horizontal line outwards for infinity (within the y bounds)
 * 2. find all intersections along that line with the polygon, store all line segements in a list
 * 3. chose a random line segement and a random point on the line
 * 
 * 
 * 
 */
 
bool sortx( const ignition::math::Vector2d& v1, 
               const ignition::math::Vector2d& v2 ) { 
 return v1.X() < v2.X(); 
} 

 
 
bool random_point(vector <ignition::math::Vector2d> polygon, vector_list table_points, ignition::math::Vector2d &result){
	
	int iterations = 0;
	bool found = false;
	ignition::math::Vector2d res;
		while (!found){
		
			double y_max = -10000, y_min = 10000;
			for (ignition::math::Vector2d point: polygon){
				y_max = max(point.Y(), y_max);
				y_min = min(point.Y(), y_min);
			}
			
			double rand_height = ignition::math::Rand::DblUniform(y_min+0.01,y_max-0.01);
			ignition::math::Line2d h_line = ignition::math::Line2d(-10000, rand_height, 10000, rand_height);
			
			//cout << rand_height << endl;
			vector <ignition::math::Vector2d> intersections;
			
			for (int i =0; i<(int)polygon.size(); i+=2){
				ignition::math::Line2d seg = ignition::math::Line2d(polygon[i], polygon[i+1]);
				ignition::math::Vector2d intersection;

				if (intersect(h_line, seg, intersection)){
					//printf("int test: (%f, %f)\n", intersection.X(), intersection.Y());
					intersections.push_back(intersection);
				}

				
			}
			
			//cout << intersections.size() << endl;
			sort(intersections.begin(), intersections.end(), sortx);
			
			
			vector<ignition::math::Line2d> lines;
			
			for (int i = 0; i<(int) intersections.size()-1; i+=2){
				ignition::math::Vector2d p1 = intersections[i];
				ignition::math::Vector2d p2 = intersections[i+1];
				ignition::math::Line2d seg = ignition::math::Line2d(p1,p2);
				lines.push_back(seg);
			}
			
			if (lines.size() <= 0){
				ROS_WARN("INVALID POLYGON: failed to add person\n");
				return false;
			}
			//cout << lines.size() <<endl;
			int rand_index = ignition::math::Rand::IntUniform(0,(int) lines.size()-1);
			
			ignition::math::Line2d rand_seg = lines[rand_index];
			
			
			///TODO: Check Buffer distance 

			res = ignition::math::Vector2d(rand_seg[0].X()+ ignition::math::Rand::DblUniform(0.3, rand_seg.Length()-0.3), rand_seg[0].Y());
			found = true;
			for (ignition::math::Vector2d v: table_points.list){
				double distance = res.Distance(v);
				//double distance = sqrt(pow((res.X()-v.X()),2) + pow((res.Y()-v.Y()),2));
				if (distance < 2){
					//cout << distance << endl;
					found = false;
					break;
				}
			}
			//printf("startpos (%f, %f)\n", res.X(), res.Y());
			iterations++;
			if (iterations >= 1000){
				ROS_WARN("TRYING TO ADD TO MANY PEOPLE: failed to add person\n");
				return false;
			}
	}
	result = res;
	return true;
}

void add_rectangle(double x1, double y1, double x2, double y2, vector <ignition::math::Vector2d> &polygon){
	add_by_points(x1,y1,x1,y2,polygon);
	add_by_points(x1,y1,x2,y1,polygon);
	add_by_points(x1,y2,x2,y2,polygon);
	add_by_points(x2,y1,x2,y2,polygon);
}




/*
 * Section struct:
 * 
 * - section boundary (in the form of a polygon)
 * - section area (double)
 * - num_people 
 * - suitable table placement points 
 * - num_tables 
 * - table model
 * - chair model
 * - LIST OF TABLES
 * - LIST OF PEOPLE
 * 
 */



struct Section{
	vector<ignition::math::Vector2d> polygon; 
	double area;
	vector_list table_points;
	string table_model;
	string chair_model;
	vector <Person> people;
	vector <Table> tables;
	
	Section(vector<ignition::math::Vector2d> _polygon, double _area, vector_list _table_points, string _table_model = "table_conference_2", string _chair_model = "chair_3"){
		
		polygon = _polygon;
		area = _area;
		table_points = _table_points;
		table_model = _table_model;
		chair_model = _chair_model;
		
		
	}
	

	void gen_tables(int num_tables){
		random_shuffle(table_points.list.begin(), table_points.list.end());
		int i;

		for (i =0; i<num_tables; i++){
			if (i < (int) table_points.list.size()){
				tables.push_back(Table(table_points.list[i], rand()%2, 1+rand()%4, table_model, chair_model));
			} else{
				break;
			}
	
				
		}
		
		while (i<(int) table_points.list.size()){
			table_points.list.pop_back();
			++i;
		}
		
	}
	
	void gen_people(int num_people){
		for (int i =0; i<num_people; i++){
			ignition::math::Vector2d point;
			if (random_point(polygon,table_points, point)){
				// currently: libboids_plugin.so or librandomwalk_plugin.so
				people.push_back(Person(point, polygon, ignition::math::Rand::DblNormal(0.9,0.15), "libboids_plugin.so"));
			}
		}
	}
	
};

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
//TODO: FIX THIS PLUGIN SYSTEM
void write_randomwalk_plugin(Person person, ofstream& out){
	 // write the bounds and starting point 
    
    out << "\t\n<plugin name=\"trajectory\" filename=\"librandomwalk_plugin.so\">\n";
    
    
    for (int i =0; i < (int) person.polygon.size(); i++){
		out << "<point>\n";
		out << person.polygon[i].X() << endl;
		out << person.polygon[i].Y() << endl;
		out << "</point>\n";
			
	}

    
	write_point(person.start.X(), person.start.Y(), 0, out, false);
    
    out << "\n<building>myhal</building>\n";
    out << "\n<max_speed>" << person.speed << "</max_speed>\n";
	out << "<obstacle_margin>0.5</obstacle_margin>\n";
	
    
    out << "</plugin>\n";
}

void write_boid_plugin(Person person, ofstream& out){
	 out << "\t\n<plugin name=\"trajectory\" filename=\"libboids_plugin.so\">\n";
    
    
    for (int i =0; i < (int) person.polygon.size(); i++){
		out << "<point>\n";
		out << person.polygon[i].X() << endl;
		out << person.polygon[i].Y() << endl;
		out << "</point>\n";
			
	}

    
	write_point(person.start.X(), person.start.Y(), 0, out, false);
    out << "<velocity>" << endl;
    out << ignition::math::Rand::DblUniform(-1.5,1.5) << endl;
    out << ignition::math::Rand::DblUniform(-1.5,1.5) << endl;
    out << 0 << endl;
    out << "</velocity>" << endl;
	out << "\n<building>myhal</building>\n";
    out << "\n<max_speed>" << person.speed*2 << "</max_speed>\n";
	out << "<obstacle_margin>0.5</obstacle_margin>\n";
	
    
    out << "</plugin>\n";
}

void write_person(Person person, ofstream& out){
	
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
	out << "<actor name=\"human_" << person.start.X() << "," << person.start.Y() << "," << rand()%10000 <<"\">\n";
	
	write_point(person.start.X(), person.start.Y(), 0, out, false);
	
	out << "\n\t<skin>\n\t<filename>model://actor/meshes/SKIN_man_" << color << "_shirt.dae</filename>\n\t</skin>\n";
	
	out << "\t<animation name=\"animation\"> \n\t\t<filename>model://actor/meshes/ANIMATION_walking.dae</filename>\n\t\t<interpolate_x>true</interpolate_x>\n\t</animation>\n";
    
    
    // write the bounds and starting point 
    
    if (person.plugin_name.compare("librandomwalk_plugin.so") == 0){
		write_randomwalk_plugin(person, out);
	} else if (person.plugin_name.compare("libboids_plugin.so") == 0){
		write_boid_plugin(person, out);
	}
    
    
    out << "</actor>\n";
	
}

void write_chair(double x, double y, double angle, ofstream& out, string chair_model = "chair_3"){
	
		out << endl;
		
		out << "<include>\n";
		out << "<uri>model://"<< chair_model << "</uri> \n";
		out << "<name> \"" << "chair_" << x << y << rand()%100 << "\" </name> \n";

		//cout << angle_shift << endl;
		out << "<pose> " << x << " " << y << " " << 0 << " " << 0 << " " << 0 << " " << angle  << " </pose>\n";
		
		out << "</include>\n";
	
		out << endl;
}

void write_table(Table table, ofstream& out){
	
	out << endl;
	out << "<include>\n";
    out << "<uri>model://" << table.table_model << "</uri> \n";
    out << "<name> \"" << "table_" << table.pos.X() << table.pos.Y() << "\" </name> \n";
    out << "<pose> " << table.pos.X() << " " << table.pos.Y() << " " << 0 << " " << 0 << " " << 0 << " " << 1.57*((int) table.vert) << " </pose>\n";
    out << "</include>\n";
    
    for (int i = 0; i< table.num_chairs; i++){
		
		/*
		 * Note:
		 * if (i&1) == 1, then we are on the left of the table
		 * if (i&2) == 2, then we are on the bottom of the table
		 * 
		 */
		 
		double x_shift = 0.5*((double) rand() / (RAND_MAX))+0.6; //0.5 to 1.1 meter way from the center of the table 
		double y_shift = 0.5*((double) rand() / (RAND_MAX))+0.6; //0.5 to 1.1 meter way from the center of the table 
		
		
		if (table.vert){
			double angle_shift = (x_shift)*3.1415*0.5*(-1+2*((double) rand() / (RAND_MAX))); // we must scale the allowable rotation shift by the distance from the table 
			write_chair(table.pos.X() + x_shift-2*x_shift*(i&1), table.pos.Y() + y_shift-2*y_shift*((i&2) == 2), angle_shift + (((int) table.vert) == 0)*((i&2)==2)*3.14 + (((int) table.vert))*((i&1)*3.14-1.57), out, table.chair_model);
		} else{
			double angle_shift = (y_shift)*3.1415*0.5*(-1+2*((double) rand() / (RAND_MAX))); // we must scale the allowable rotation shift by the distance from the table 
			write_chair(table.pos.X() + + x_shift-2*x_shift*(i&1), table.pos.Y() + y_shift-2*y_shift*((i&2) == 2), angle_shift + (((int) table.vert) == 0)*((i&2)==2)*3.14 + (((int) table.vert))*((i&1)*3.14-1.57), out, table.chair_model);	
		}
		

	}
}



void write_file(ifstream& in, ofstream& out, vector <Section> sections){//vector <Person> people, vector <Table> tables){
	char str[255];
	int line =0;

	while(in) {
		in.getline(str, 255);  // delim defaults to '\n'
		if(in) {
			
			if (line == 105){
				// insert writing people and furnature here
				
				for (Section section: sections){
					for (Person person: section.people){
						write_person(person, out);
					}
					
					for (Table table: section.tables){
						write_table(table, out);
					}
				}
			}

			out << str << endl;
			line++;

		}
	}	
	
}
