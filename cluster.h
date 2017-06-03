#ifndef CLUSTER_H
#define CLUSTER_H


#include <stdlib.h>
#include <string.h> 
#include <vector>
#include <map>
#include <list>

#define RSHAPE 0
#define OBSTACLE 1

struct Coords;
struct Shape;
struct Via;
class Cluster;

using namespace std;

typedef multimap< int , Shape* , less<int> > MAP_Shape;
typedef MAP_Shape::iterator MS_it;
typedef list<Shape*>::iterator it_shape;
typedef pair<int, int> Point;


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

};

struct Via{
	int x;
	int y;
};




class Cluster{
	
public:
	Cluster(Shape*);
	void Add_shape(Shape*);


private:
	bool Shape_type;//Rshape or Obstacle
	int shape_num;
	list < Shape* > shape_list;
	list < Point* > boundary; //counterclkwise

	Point *TopLCorner;
	Point *TopRCorner;
	Point *DownLCorner;
	Point *DOwnRCorner;

	//int

};





#endif
