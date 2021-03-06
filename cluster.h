#ifndef CLUSTER_H
#define CLUSTER_H


#include <stdlib.h>
#include <string.h> 
#include <vector>
#include <map>
#include <list>
#include <climits>

#include "fiboheap.h"


#define RSHAPE   0
#define OBSTACLE 1
#define VIA 	 2

#define LEFT  0
#define RIGHT 1

//#define LEFT  1
//#define RIGHT 2
#define UP    4
#define DOWN  8

struct Coords;
class Shape;
struct Line;
class GraphPoint;
class BoundLine_info;
struct Edge_info;
class Cluster;

using namespace std;

typedef pair<int, int> Point;
typedef multimap< int , Shape* , less<int> > MAP_Shape;
typedef map< int , Edge_info* , less<int> > MAP_GP_edge; //Point: this cluster point
//typedef multimap< int , GraphPoint*, less<int> > MAP_GP_edge;
typedef pair<map< int , Edge_info* , less<int> >::iterator,bool> MAP_GP_Status;
typedef MAP_Shape::iterator MS_it;
typedef list<Shape*>::iterator it_shape;


struct Coords{
	int x1;
	int y1;
	int x2;
	int y2;
};


class Shape{
public:
	Shape(int, int);
	Coords *coords;
	int Shape_type;//Rshape or Obstacle or Via
	int layer_position;//0, 1, 2 ...
	Cluster *clu;
    list < Shape* > Overlaps;
};


struct Line{
	int x;
	int y;
	int length;
	int width;
	bool LR; //LEFT or RIGHT
	Shape *S;
};

class GraphPoint{//Only consider same layer
public:
	GraphPoint(Line*, int, int);
	GraphPoint(int, int, int);
	GraphPoint(int, int, int, int&);
	GraphPoint(int, int, int, int&, Cluster*);
	void Add_edge(GraphPoint*, int, int, int, int, int, int);
	GraphPoint* Find_Set();
	
	Cluster *clu;
	int Shape_type;
	int Layer_pos;
	MAP_GP_edge map_edge;
	
	//if the GraphPoint is RoutedShape, x & y is not a fixed value
	int x;
	int y;
	
	int idx;


	//Extended Dijkstra's
	FibHeap<int>::FibNode *Fnode;
	GraphPoint *parent;//no null
	GraphPoint *root; //no null
	int terminal_dis; // Extended Dijkstra's & Opt1 use & prims
	bool select;

	//Extended Kruskal's
	GraphPoint *parentKK;
	bool visit;
	unsigned rank;

	//final edge
	list<Edge_info*> final_edge;
	list<Edge_info*> ftemp_edge;
	GraphPoint *path; //Opt1
	GraphPoint *path_opt; //Opt1

	//Extended Prim's
	list<GraphPoint*> prims_tree_nd;
	bool prims_select;
	int prims_weight;
	Edge_info *prims_edge;

	//test
	static int construct_edge;
	static int construct_min_edge;
};


class BoundLine_info{
public:
	BoundLine_info(int m_x, int temp_x, int flag, int P_y, int min_x, GraphPoint *gpp);//Max_x, point_x, flag(UP_or_down), point_y, min_x
	BoundLine_info(int max_x, int P_y, int min_x, int down_x); //0815 for R_bound_map
	int Get_up_edge_x()  { return up_edge_x;}
	int Get_down_edge_x(){ return down_edge_x;}
	void Change_up_edge(GraphPoint* gp1, int _x){
		//if(gp1!=NULL && gp1->Shape_type!=RSHAPE){ cout << "error!";}
		up_edge_GP = gp1;
		up_edge_x = _x;
	}
	void Change_down_edge(GraphPoint* gp1, int _x){
		//if(gp1!=NULL && gp1->Shape_type!=RSHAPE){ cout << "error!";}
		down_edge_GP = gp1;
		down_edge_x = _x;
	}
	GraphPoint* Gp;
	int max_x;
	int point_x;
	int point_y;

	int up_edge_x;
	GraphPoint* up_edge_GP;
	int down_edge_x;
	GraphPoint* down_edge_GP;


};

struct Edge_info{
	Edge_info(GraphPoint* gp, int x, int y, int x2, int y2, int d, int l):Gp(gp),point_x1(x),point_y1(y),point_x2(x2),point_y2(y2),layer(l),distance(d)
	{
	}

	GraphPoint* Gp;
	int point_x1;
	int point_y1;
	int point_x2;
	int point_y2;
	int layer;

	int distance;

	//Extended Kruskal's
	GraphPoint* source;
};

struct Edge
{
	Edge(int x1, int x2, int y1, int y2) {
		_x1 = x1; _x2 = x2; _y1 = y1; _y2 = y2;
	}
	int _x1, _x2, _y1, _y2;
};

class Cluster{
	friend class Layer;
public:
	Cluster(Shape*);
	Cluster(int);
	void Add_shape(Shape*);
	GraphPoint* Add_GP(Line*, int, int &);
	GraphPoint* Add_GP(int, int, int, int &);
	int GetShapeType();
//private:
	int Shape_type;//Rshape or Obstacle or via
	int shape_num;
	list < Shape* > shape_list;
	list < GraphPoint* > GraphP_list; //routed shape cluster only one GP

};


struct via_pos{
	via_pos(int _x, int _y, int _vial, bool ud):x(_x),y(_y),via_layer(_vial),up_or_down(ud){}// for checker
	//via_pos(int _x, int _y, int via_l1, int via_l2)
	int x;
	int y;
	int via_layer;
	bool up_or_down; //up = true
	int via_num; //
};

struct Ex_prims
{
	GraphPoint *master_gp;
	Edge_info* Edge;
};

/*
Edge_info2
2017/11/02
Only store the shortest edge in each region for each GraphPoint 
So we need a data structrue (Edge_info2) to store this infomation

*/
class Edge_info2{ //store 2 edges
public:
	Edge_info2()
	{
		Gp[0] = Gp[1] = NULL;
		point_x1[0] = point_y1[0] = point_x2[0] = point_y2[0] = layer[0] = via_cost[0] = 0;
		point_x1[1] = point_y1[1] = point_x2[1] = point_y2[1] = layer[1] = via_cost[1] = 0;
		distance[0] = distance[1] = INT_MAX;
	}
	void update(GraphPoint *main_gp, GraphPoint* gp, int x, int y, int x2, int y2, int l, int v_cost){
		int temp_dis = abs(x-x2) + abs(y-y2) + v_cost;
		
		//map_temp_itr = main_gp->map_edge.find(gp->idx);
		//if(map_temp_itr!=main_gp->map_edge.end() &&  map_temp_itr->second->distance <= temp_dis) return; // runtime go up
		
		for(int i = 1;i>=0;i--){
			if(temp_dis >= distance[i]) break;
			if(i==0){
				Gp[1] = Gp[0];
				point_x1[1] = point_x1[0];
				point_y1[1] = point_y1[0];
				point_x2[1] = point_x2[0];
				point_y2[1] = point_y2[0];
				layer[1] 	= layer[0];
				via_cost[1] = via_cost[0];
				distance[1] = distance[0];
			}
			Gp[i] = gp;
			point_x1[i] = x;
			point_y1[i] = y;
			point_x2[i] = x2;
			point_y2[i] = y2;
			layer[i] = l;
			via_cost[i] = v_cost;
			distance[i] = temp_dis;
		}
	}
	void Init(){
		Gp[0] = Gp[1] = NULL;
		point_x1[0] = point_y1[0] = point_x2[0] = point_y2[0] = layer[0] = via_cost[0] = 0;
		point_x1[1] = point_y1[1] = point_x2[1] = point_y2[1] = layer[1] = via_cost[1] = 0;
		distance[0] = distance[1] = INT_MAX;
	}
	void Add_min_edge(GraphPoint* main_gp){
		if(Gp[0]!=NULL) main_gp->Add_edge(Gp[0], point_x1[0], point_y1[0], point_x2[0], point_y2[0], layer[0], via_cost[0]);
		if(Gp[1]!=NULL) main_gp->Add_edge(Gp[1], point_x1[1], point_y1[1], point_x2[1], point_y2[1], layer[1], via_cost[1]);
		Init();
	}
private:
	MAP_GP_edge::iterator map_temp_itr;
	GraphPoint* Gp[2];
	int point_x1[2];
	int point_y1[2];
	int point_x2[2];
	int point_y2[2];
	int layer[2];
	int via_cost[2];

	int distance[2];

};



/*class Edge_info2{
public:
	Edge_info2()
	{
		Gp = NULL;
		point_x1 = point_y1 = point_x2 = point_y2 = layer = via_cost = 0;
		distance = INT_MAX;
	}
	void update(GraphPoint *main_gp, GraphPoint* gp, int x, int y, int x2, int y2, int l, int v_cost){
		int temp_dis = abs(x-x2) + abs(y-y2) + via_cost;
		
		map_temp_itr = main_gp->map_edge.find(gp->idx);
		if(map_temp_itr!=main_gp->map_edge.end() &&  map_temp_itr->second->distance <= temp_dis) return; // runtime go up

		if(temp_dis < distance){
			Gp = gp;
			point_x1 = x;
			point_y1 = y;
			point_x2 = x2;
			point_y2 = y2;
			layer = l;
			via_cost = v_cost;
			distance = temp_dis;
		}
	}
	void Init(){
		Gp = NULL;
		point_x1 = point_y1 = point_x2 = point_y2 = layer = via_cost = 0;
		distance = INT_MAX;
	}
	void Add_min_edge(GraphPoint* main_gp){
		if(Gp!=NULL)
			main_gp->Add_edge(Gp, point_x1, point_y1, point_x2, point_y2, layer, via_cost);
		Init();
	}
private:
	MAP_GP_edge::iterator map_temp_itr;
	GraphPoint* Gp;
	int point_x1;
	int point_y1;
	int point_x2;
	int point_y2;
	int layer;
	int via_cost;

	int distance;
};*/



#endif
