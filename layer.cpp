#include "layer.h"
#include <iostream>

using namespace std;


Layer::Layer()
:Rshape_num(0),Obstacle_num(0),Via_num(0) //initialize
{
	//cout << "QQ";
}

void Layer::SpanningGraphConstruct(){
	/* 1.1 sorting
	   1.2 construct cluster

	   */


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