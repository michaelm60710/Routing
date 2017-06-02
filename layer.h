#ifndef LAYER_H
#define LAYER_H


#include "cluster.h"

using namespace std;



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
	vector <Shape* > all_shape_vec;
	//vector <Shape* > X_sort_shape;
	MAP_Shape X_msort_shape; //sort X2
	MAP_Shape Y_msort_shape; //sort Y2
	/*vec < Shape* > Rshape_vec;
	vec < Shape* > Obstacle_vec;
	vec < Via*   > Via_vec;*/
	int Rshape_num;
	int Obstacle_num;
	int Via_num;
	int Layer_Shape_num;
	//step 1



};







#endif
