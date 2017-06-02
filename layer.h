#ifndef LAYER_H
#define LAYER_H



#include <stdlib.h>
#include <string.h> 
#include <vector>
#include <list>

#define RSHAPE 0
#define OBSTACLE 1

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
	void SpanningGraphConstruct();            //step 1
	void SpanningTreeConstruct();             //step 2
	void RectilinearSpanningTreeConstruct();  //step 3
	void OARSMT();                            //step 4


	//for private use
	void Rshape_list_append(Shape *);
	void Obstacle_list_append(Shape *);
	void Via_list_append(Via *);
	int get_Rshape_num();
	int get_Obstacle_num();
	int get_Via_num();

private:
	list < Shape* > Rshape_list;
	list < Shape* > Obstacle_list;
	list < Via*   > Via_list;
	vector <Shape* > X_sort_shape;
	/*vec < Shape* > Rshape_vec;
	vec < Shape* > Obstacle_vec;
	vec < Via*   > Via_vec;*/
	int Rshape_num;
	int Obstacle_num;
	int Via_num;
	int Layer_Shape_num;
	//step 1



};



typedef list<Shape*>::iterator it_shape;




#endif
