#include "cluster.h"
#include <iostream>
#include <algorithm>


Cluster::Cluster(Shape *temp_shape)
{
	shape_num = 1;
	Shape_type = temp_shape->Shape_type;
	shape_list.push_back(temp_shape);
	TopLCorner  = new Point(temp_shape->x1,temp_shape->y2);
	TopRCorner  = new Point(temp_shape->x2,temp_shape->y2);
	DownLCorner = new Point(temp_shape->x1,temp_shape->y1);
	DOwnRCorner = new Point(temp_shape->x2,temp_shape->y1);
	boundary.push_back(DownLCorner);
	boundary.push_back(DOwnRCorner);
	boundary.push_back(TopRCorner);
	boundary.push_back(TopLCorner);

}