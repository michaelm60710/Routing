#include "cluster.h"
#include <iostream>
#include <algorithm>


Shape::Shape(int shape_type, int layer_pos):Shape_type(shape_type),layer_position(layer_pos)
{}



Cluster::Cluster(Shape *temp_shape)
{
	shape_num = 1;
	Shape_type = temp_shape->Shape_type;
	shape_list.push_back(temp_shape);
}

Cluster::Cluster(int _shape_type)
{
	shape_num = 0;
	Shape_type = _shape_type;
}

void Cluster::Add_shape(Shape *temp_shape){


}

int Cluster::GetShapeType(){
	return Shape_type;
}

GraphPoint* Cluster::Add_GP(Line* L, int UPorDown, int &_idx){
	GraphPoint* gp;
	if(Shape_type==OBSTACLE || GraphP_list.size()==0){
		gp = new GraphPoint(L, UPorDown, _idx++);
		GraphP_list.push_back(gp);
	}
	else{ //ROUTED SHAPE & already exist
		gp = (*GraphP_list.begin());
	}

	//check
	if(Shape_type==RSHAPE && GraphP_list.size()>1){
		cerr << "bug";
		//cin.get();
	}
	return gp;
}

GraphPoint* Cluster::Add_GP(int pos_x, int pos_y, int layer, int &_idx){// for Extra via(Obs)
	GraphPoint* gp = new GraphPoint(layer, pos_x, pos_y, _idx, this);
	GraphP_list.push_back(gp);

	return gp;
}

int GraphPoint::construct_edge = 0;
int GraphPoint::construct_min_edge = 0;

GraphPoint::GraphPoint(Line* L, int UPorDown, int _idx){
	clu = L->S->clu;
	Shape_type = L->S->Shape_type;
	Layer_pos = L->S->layer_position;
	x = L->x;
	if(UPorDown & UP){ //UP = true
		y = L->y + L->length;
	}
	else{ //Down = false
		y = L->y;
	}
	idx = _idx;
	// initialization for Extended Kruskal's
	visit = false;
	rank = 0;
}

GraphPoint::GraphPoint(int _layer_pos, int _x, int _y){ // for opt1
	clu = NULL;
	Layer_pos = _layer_pos;
	idx = -1;
	x = _x;
	y = _y;
	visit = false;
	select = false;
	rank = 0;
	terminal_dis = INT_MAX;
	Shape_type = OBSTACLE; //Output file need to use QQ
}

GraphPoint::GraphPoint(int _layer_pos, int _x, int _y, int &_idx){ // for SGC_2
	clu = NULL;
	Layer_pos = _layer_pos;
	idx = _idx++;
	x = _x;
	y = _y;
	visit = false;
	select = false;
	rank = 0;
	terminal_dis = INT_MAX;
	Shape_type = OBSTACLE;
}

GraphPoint::GraphPoint(int _layer_pos, int _x, int _y, int &_idx, Cluster *_clu){ // for extra obstacle
	clu = _clu;
	Shape_type = OBSTACLE;
	Layer_pos = _layer_pos;
	idx = _idx++;
	x = _x;
	y = _y;
	visit = false;
	select = false;
	rank = 0;
	terminal_dis = INT_MAX;
	Shape_type = OBSTACLE;
}


void GraphPoint::Add_edge(GraphPoint *insert_gp, int my_x, int my_y, int insert_x, int insert_y, int layer, int via_cost){ //add_dis = 0 or Viacost
	if (insert_gp->clu == clu && Shape_type==RSHAPE) return;//same cluster
	int distance = abs(my_x-insert_x) + abs(my_y-insert_y) + via_cost;
	MAP_GP_Status status1;//,status2;
	Edge_info *E1 = NULL;
	status1 = map_edge.insert(MAP_GP_edge::value_type(insert_gp->idx, E1 ) );
	if( my_x==6888 && my_y==9096 && insert_x==6168) {//(6168,9096) to (6888,9096)
		cout << "edge x y:" << insert_x << ", " << insert_y << endl;
		//cin.get();

	}
	if(status1.second==true){
		E1 = new Edge_info(insert_gp, my_x, my_y, insert_x, insert_y, distance, layer);
		status1.first->second = E1;
	}
	else{
		if (distance < status1.first->second->distance){ //update point position & distance
			status1.first->second->point_x1 = my_x;
			status1.first->second->point_y1 = my_y;
			status1.first->second->point_x2 = insert_x;
			status1.first->second->point_y2 = insert_y;
			status1.first->second->distance = distance;
			status1.first->second->layer    = layer;
			construct_min_edge++; //static
		}
	}
	construct_edge++; //static

}

GraphPoint* GraphPoint::Find_Set(){


	if(parent==NULL){//bug?
		//cout << "parent = NULL"<<endl;
		root = NULL;
		return root;
	}
	if(parent == parent->parent) root = parent;
	else 						 root = parent->Find_Set();
	return root;
}

BoundLine_info::BoundLine_info(int m_x, int temp_x, int flag, int P_y, int min_x, GraphPoint *gpp=NULL):Gp(NULL),max_x(m_x),point_x(temp_x),point_y(P_y)
{
	//min_x = spacing - 1
	up_edge_GP = down_edge_GP = NULL;
	if(flag & UP) {
		down_edge_x = m_x;
		down_edge_GP = gpp;
		up_edge_x   = min_x;
	}
	else if(flag & DOWN){
		down_edge_x = min_x;
		up_edge_x   = m_x;
		up_edge_GP = gpp;
	}
	else { //VIA
		down_edge_x = min_x;
		up_edge_x   = min_x;
	}
}

BoundLine_info::BoundLine_info(int max_x, int P_y, int min_x, int down_x):Gp(NULL),max_x(max_x),point_x(max_x),point_y(P_y){

	up_edge_GP = down_edge_GP = NULL;
	down_edge_x = down_x;
	up_edge_x   = min_x;

}
