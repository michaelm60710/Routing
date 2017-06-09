#ifndef CLUSTER_H
#define CLUSTER_H


#include <stdlib.h>
#include <string.h> 
#include <vector>
#include <map>
#include <list>
#include <climits>

#include "fiboheap.h"
#include "fiboqueue.h"


#define RSHAPE 0
#define OBSTACLE 1
#define LEFT 0
#define RIGHT 1

//#define LEFT  1
//#define RIGHT 2
#define UP    4
#define DOWN  8

struct Coords;
struct Shape;
struct Via;
struct Line;
class GraphPoint;
struct BoundLine_info;
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


struct Shape{

	//int x;
	//int y;
	//int width;
	//int height;
	Coords *coords;
	bool Shape_type;//Rshape or Obstacle
	Cluster *clu;
    int    y_idx;
    list < Shape* > Overlaps;
};

struct Via{
	int x;
	int y;
};

struct Line{
	int x;
	int y;
	int length;
	bool LR; //LEFT or RIGHT
	Shape *S;
};

class GraphPoint{
public:
	GraphPoint(Line*, int, int);
	void Add_edge(GraphPoint*, int, int, int, int);
	GraphPoint* Find_Set();
	Cluster *clu;
	bool Shape_type;
	MAP_GP_edge map_edge;
	//if the GraphPoint is RoutedShape, x & y is not a fixed value
	int x;
	int y;
	
	int idx;


	//Extended Dijkstra's
	FibHeap<int>::FibNode *Fnode;
	GraphPoint *parent;
	GraphPoint *root;
	int terminal_dis;
	bool select;
};

struct BoundLine_info{
	BoundLine_info(bool lr, int m_x, int temp_x, int flag, int P_y):Gp(NULL),LR(lr),max_x(m_x),point_x(temp_x),pos_flag(flag),point_y(P_y)
	{
		if(flag & UP) {
			down_edge_x = m_x;
			up_edge_x   = -1;
		}
		else{
			down_edge_x = -1;
			up_edge_x   = m_x;
		}
	}
	
	GraphPoint* Gp;
	bool LR; // no use
	int max_x;
	int point_x;
	int pos_flag; // no use
	int point_y;
	int up_edge_x;
	int down_edge_x;


};

struct Edge_info{
	Edge_info(GraphPoint* gp, int x, int y, int x2, int y2, int d):Gp(gp),point_x1(x),point_y1(y),point_x2(x2),point_y2(y2),distance(d)
	{
	}

	GraphPoint* Gp;
	int point_x1;
	int point_y1;
	int point_x2;
	int point_y2;

	int distance;
};


class Cluster{
	friend class Layer;
public:
	Cluster(Shape*);
	void Add_shape(Shape*);
	void Add_GP(GraphPoint*);
	GraphPoint* Add_GP(Line*, int, int &);

	bool GetShapeType();
private:
	bool Shape_type;//Rshape or Obstacle
	int shape_num;
	list < Shape* > shape_list;
	list < Point* > boundary; //counterclkwise
	list < GraphPoint* > GraphP_list; //routed shape cluster only one GP

	Point *TopLCorner;
	Point *TopRCorner;
	Point *DownLCorner;
	Point *DOwnRCorner;

	//int

};





#endif
