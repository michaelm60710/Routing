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

}

void GraphPoint::Add_edge(GraphPoint *insert_gp){
	if (insert_gp->clu == clu) return;
	map_edge.insert(MAP_GP_edge::value_type(insert_gp->idx, insert_gp) );


}

