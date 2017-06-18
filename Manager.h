#ifndef Mgr_H
#define Mgr_H

#include <fstream>
#include "layer.h"

using namespace std;



class Manager
{
public:
	Manager(const char* ,const char* );


	void Parsing(const char* );
	Coords* Parsing_coordinate(string, string);
	Coords* Parsing_via(string);

	void SpanningGraphConstruct();

private:
	int ViaCost;
	int Spacing;
	Coords *Boundary;
	/*int Boundary_x1;
	int Boundary_y1;
	int Boundary_x2;
	int Boundary_y2;*/
	int MetalLayers;
	int RoutedShapes;
	int RoutedVias;
	int Obstacles;
	vector < Layer > all_layer;
	
	vector < Shape* > all_shape;//via, obstacle, rshape
	vector <Line* > all_line;


};









#endif
