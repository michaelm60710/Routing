#include "cluster.h"
#include <iostream>
#include <algorithm>


Cluster::Cluster(Shape *temp_shape)
{
	shape_num = 1;
	Shape_type = temp_shape->Shape_type;
	shape_list.push_back(temp_shape);
	TopLCorner  = new Point(temp_shape->coords->x1,temp_shape->coords->y2);
	TopRCorner  = new Point(temp_shape->coords->x2,temp_shape->coords->y2);
	DownLCorner = new Point(temp_shape->coords->x1,temp_shape->coords->y1);
	DOwnRCorner = new Point(temp_shape->coords->x2,temp_shape->coords->y1);
	boundary.push_back(DownLCorner);
	boundary.push_back(DOwnRCorner);
	boundary.push_back(TopRCorner);
	boundary.push_back(TopLCorner);

}

void Cluster::Add_shape(Shape *temp_shape){


}

void Cluster::Add_GP(GraphPoint *gp){
	GraphP_list.push_back(gp);
}

bool Cluster::GetShapeType(){
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
		cin.get();
	}

	return gp;
}



GraphPoint::GraphPoint(Line* L, int UPorDown, int _idx){
	clu = L->S->clu;
	Shape_type = L->S->Shape_type;
	x = L->x;
	if(UPorDown & UP){ //UP = true
		y = L->y + L->length;
	}
	else{ //Down = false
		y = L->y;
	}
	idx = _idx;
	// initialization for Extended Kruskal's
	visit = chosen = false;
	rank = 0;
}

void GraphPoint::Add_edge(GraphPoint *insert_gp, int my_x, int my_y, int insert_x, int insert_y){
	if (insert_gp->clu == clu && clu->GetShapeType()==RSHAPE) return;//same cluster
	int distance = abs(my_x-insert_x) + abs(my_y-insert_y);
	MAP_GP_Status status1;//,status2;
	Edge_info *E1;//, *E2;
	//E1 = new Edge_info(insert_gp, my_x, my_y, insert_x, insert_y, distance);
	status1 = map_edge.insert(MAP_GP_edge::value_type(insert_gp->idx, E1 ) );
	//check
	/*if(status1.second != status2.second){
		cerr << "bug";
		cin.get();
	}*/

	if(status1.second==true){
		E1 = new Edge_info(insert_gp, my_x, my_y, insert_x, insert_y, distance);
		status1.first->second = E1;
	}
	else{
		if (distance < status1.first->second->distance){ //update point position & distance
			status1.first->second->point_x1 = my_x;
			status1.first->second->point_y1 = my_y;
			status1.first->second->point_x2 = insert_x;
			status1.first->second->point_y2 = insert_y;
			status1.first->second->distance = distance;
		}
	}

}

GraphPoint* GraphPoint::Find_Set(){


	if(parent==NULL){//bug?
		cout << "parent = NULL"<<endl;
		root = NULL;
		return root;
	}
	if(parent == parent->parent) root = parent;
	else 						 root = parent->Find_Set();
	return root;
}

