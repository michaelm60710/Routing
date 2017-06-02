#ifndef LAYER_H
#define LAYER_H



#include <stdlib.h>
#include <string.h> 
#include <vector>
#include <list>

#define RSHAPE 0
#define ObSTACLE 1

using namespace std;


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
};

struct Via{
	int x;
	int y;
};







class Layer{
public:
	Layer();

//private:
	list < Shape* > Rshape_list;
	list < Shape* > Obstacle_list;
	list < Via*   > Via_list;

	//step 1



};




#endif
