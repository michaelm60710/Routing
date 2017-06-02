#ifndef Mgr_H
#define Mgr_H

#include <fstream>
#include <stdlib.h>
#include <string.h> 
#include <vector>
#include <list>

#include "layer.h"
using namespace std;



class Manager
{
public:
	Manager(const char* ,const char* );


	void Parsing(const char* );
	Coords* Parsing_coordinate(string, string);
	Via* Parsing_via(string);

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


};









#endif
