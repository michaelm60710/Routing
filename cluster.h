#ifndef CLUSTER_H
#define CLUSTER_H


#include <stdlib.h>
#include <string.h> 
#include <vector>
#include <map>
#include <list>

#define RSHAPE 0
#define OBSTACLE 1
#define LEFT 0
#define RIGHT 1

struct Coords;
struct Shape;
struct Via;
struct Line;
struct GraphPoint;
struct BoundLine_info;
class Cluster;

using namespace std;

typedef pair<int, int> Point;
typedef multimap< int , Shape* , less<int> > MAP_Shape;
typedef multimap< int , pair<GraphPoint*, Point*> , less<int> > MAP_GP_edge;
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
	GraphPoint(Line*, bool);
	Cluster *clu;
	bool Shape_type;
	MAP_GP_edge map_edge;
	//if the GraphPoint is RoutedShape, x & y is not a fixed value
	int x;
	int y;


};

struct BoundLine_info{
	GraphPoint* Gp;
	bool LR;
	int max_x;

};


class Cluster{
	friend class Layer;
public:
	Cluster(Shape*);
	void Add_shape(Shape*);


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
