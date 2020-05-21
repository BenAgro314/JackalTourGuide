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

#include <ros/ros.h>


using namespace std;





struct WayPoint{
	double x,y;
	int id;
	bool filled; // is the waypoint currently filled with furnature or walls
	bool furb; // is the waypoint in a suitable location to be filled with furnature 
	int row, col;
	WayPoint(double ax = 0, double ay =0, bool fu = true, bool fi = false, int i =-1){
		x = ax;
		y = ay;
		id = i;
		furb = fu;
		filled = fi;
	}
	
};

struct Graph{

	vector<vector<WayPoint *>> adj_list;
	map <int, WayPoint *> points;
	
	Graph(){
		
	}
	
	void add_node(WayPoint * A){
		A->id = (int) adj_list.size();
		points[A->id] = A;
		
		
		vector <WayPoint *> nebs;
		adj_list.push_back(nebs);
	}
	
	void add_edge(WayPoint * A, WayPoint * B, bool dir = true){
	
		adj_list[A->id].push_back(B);
		
		if (!dir){
			adj_list[B->id].push_back(A);
			
		}
	}
	
	
	void print_graph(){

		
		for (int i =0; i< (int) adj_list.size(); i++){
			printf("[PARENT: id: %d, (%f,%f), %d] -> ", i, points[i]->x, points[i]->y, points[i]->filled);
			for (int j =0; j< (int) adj_list[i].size(); j++){
				printf("[id: %d, (%f,%f), %d] -> ", adj_list[i][j]->id, adj_list[i][j]->x, adj_list[i][j]->y, adj_list[i][j]->filled);
			}
			printf("NULL\n");
		}
				
	}
};

double angle_finder(WayPoint A, WayPoint B){
	
	double dy = B.y - A.y;
	double dx = B.x - A.x;
	double ans = atan2(dy , dx);
	return ans;
}

double dist(WayPoint A, WayPoint B){
	
	return sqrt(pow(A.x-B.x,2) + pow((A.y-B.y),2));
}


void add_by_id(Graph& G, int a, int b){
	WayPoint * A = G.points[a];
	WayPoint * B = G.points[b];
	G.add_edge(A,B);
}

Graph add_graphs(Graph G1, Graph G2, vector<vector<int>> new_edges){ //new edges specifies the id of the waypoint in the first graph that is connected to the id of the waypoint in the second 
	// first we need to modify all of the id's of the second graph such that their adjacency lists can be concatinated 
	Graph res = Graph();
	
	
	for (int i =0; i< (int) G1.adj_list.size(); i++){
		WayPoint * parent = G1.points[i];
		vector <WayPoint *> neighbours = G1.adj_list[i];
		parent->id = res.adj_list.size();
		res.points[parent->id] = parent;
		res.adj_list.push_back(neighbours);
	}
	
	for (int i =0; i< (int) G2.adj_list.size(); i++){
		WayPoint * parent = G2.points[i];
		vector <WayPoint *> neighbours = G2.adj_list[i];
		parent->id = res.adj_list.size();
		res.points[parent->id] = parent;
		res.adj_list.push_back(neighbours);
	}
	
	//res.print_graph();
	
	for (vector<int> edge: new_edges){
		add_by_id(res, edge[0], edge[1]+G1.adj_list.size());
		//cout << "INSIDE" << endl;
		add_by_id(res, edge[1]+G1.adj_list.size(), edge[0]);
	}
	
	return res;
}
