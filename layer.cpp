#include "layer.h"
#include <iostream>
#include <algorithm>

using namespace std;


Layer::Layer()
:Rshape_num(0),Obstacle_num(0),Via_num(0),Layer_Shape_num(0) //initialize
{
	//cout << "QQ";
}

bool Sort_Shape(Shape* a, Shape* b){
	return a->coords->x1 < b->coords->x1;
}



void Layer::SpanningGraphConstruct(){
	/* 1.1 sorting
	   1.2 construct cluster

	   */
	//#1.1 sorting x 
	int pos = 0;
	Layer_Shape_num = Rshape_num+Obstacle_num;
	X_sort_shape.resize(Layer_Shape_num);
	for (it_shape it_s= Rshape_list.begin();it_s != Rshape_list.end();++it_s)
		X_sort_shape[pos++] = (*it_s); 
	for (it_shape it_s= Obstacle_list.begin();it_s != Obstacle_list.end();++it_s)
		X_sort_shape[pos++] = (*it_s); 

	sort(X_sort_shape.begin(),X_sort_shape.end(),Sort_Shape);
	//for (int i=0;i<Rshape_num+Obstacle_num;i++)
	//	cout << i << ": " << X_sort_shape[i]->coords->x1 << endl;

	

}


































void Layer::Rshape_list_append(Shape *temp_shape){
	Rshape_num++;
	Rshape_list.push_back(temp_shape);
}

void Layer::Obstacle_list_append(Shape *temp_shape){
	Obstacle_num++;
	Obstacle_list.push_back(temp_shape);
}

void Layer::Via_list_append(Via *temp_via){
	Via_num++;
	Via_list.push_back(temp_via);
}

int Layer::get_Rshape_num(){
	return Rshape_num;
}

int Layer::get_Obstacle_num(){
	return Obstacle_num;
}

int Layer::get_Via_num(){
	return Via_num;
}