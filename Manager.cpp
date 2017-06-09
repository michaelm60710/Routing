#include "Manager.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;


Manager::Manager(const char* Input_file,const char* Output_file){

	//Parsing
	Parsing(Input_file);

	//test
	all_layer[0].SpanningGraphConstruct();
	all_layer[0].SpanningTreeConstruct();
	/*for(size_t s = 0; s < all_layer.size(); s++){
		cout<<"layer "<<s+1<<endl;
		all_layer[s].SpanningGraphConstruct();
	}*/


}



void Manager::Parsing(const char* Input_file){
	cout << "Read Data:" << endl;
	string garbage,layer,coor1,coor2;
	int l;
	ifstream i_file;

	i_file.open(Input_file,ios::in);
	//### 1. read parameters
	i_file>>garbage>>garbage>>ViaCost;
	i_file>>garbage>>garbage>>Spacing;
	i_file>>garbage>>garbage>>coor1>>coor2;
	Boundary = Parsing_coordinate(coor1,coor2);
	cout << "boundary: " << Boundary->x1 << " " << Boundary->y1 << ", " << Boundary->x2 << " " << Boundary->y2 <<endl;
	i_file>>garbage>>garbage>>MetalLayers;
	i_file>>garbage>>garbage>>RoutedShapes;
	i_file>>garbage>>garbage>>RoutedVias;
	i_file>>garbage>>garbage>>Obstacles;

	//###1.2 construct
	all_layer.resize(MetalLayers);

	//###2. read RoutedShape
	cout << "read RoutedShape..." << endl;
	for(int i = 0; i < RoutedShapes;i++){
		i_file>>garbage>>layer>>coor1>>coor2;//ex: RoutedShape>> M1>> (6469,2552)>> (6504,2558)
		Shape *temp_shape = new Shape;
		l = int(layer[1]) - 49; // M1 -> l = 0, M2 -> l = 1
		temp_shape->coords = Parsing_coordinate(coor1,coor2);
		temp_shape->Shape_type = RSHAPE;
		all_layer[l].Rshape_list_append(temp_shape);
	}

	//###3. read RoutedVias
	cout << "read RoutedVias..." << endl;
	for(int i = 0; i < RoutedVias;i++){
		i_file>>garbage>>layer>>coor1;
		l = int(layer[1]) - 49; // V1 -> 0, V2 -> 1
		Via* temp_via = Parsing_via(coor1);
		all_layer[l].Via_list_append(temp_via);
	}

	//###2. read Obstacles
	cout << "read Obstaclese..." << endl;
	for(int i = 0; i < Obstacles;i++){
		i_file>>garbage>>layer>>coor1>>coor2;
		Shape *temp_shape = new Shape;
		l = int(layer[1]) - 49; // M1 -> 0, M2 -> 1
		temp_shape->coords = Parsing_coordinate(coor1,coor2);
		temp_shape->Shape_type = OBSTACLE;
		all_layer[l].Obstacle_list_append(temp_shape);
	}



	//###3. print & check
	cout << "\nViaCost: " << ViaCost << endl;
	cout << "Spacing: " << Spacing << endl;
	cout << "MetalLayers: " <<MetalLayers << endl;
	cout << "RoutedShapes: " <<RoutedShapes << endl;
	cout << "RoutedVias: " <<RoutedVias << endl;
	cout << "Obstacles: " <<Obstacles << endl;
	for(int i=0;i<MetalLayers;i++){
		cout << "LAYER " << i << ":"<<endl;
		cout << "   Rshape size: " << all_layer[i].get_Rshape_num() << endl;
		cout << "   Via size: " << all_layer[i].get_Via_num() << endl;
		cout << "   Obstacle size: " << all_layer[i].get_Obstacle_num() << endl;
	}

}

Coords* Manager::Parsing_coordinate(string coor1, string coor2){ //ex: (4159,2948) (4263,2964)

	int comma;
	stringstream ss;
	Coords *temp_coords = new Coords;
	for(comma = 1;coor1[comma] != ','; comma++);
	ss << coor1.substr(1,comma-1) << ' ';
	ss << coor1.substr(comma+1,coor1.length()-comma-2) << ' ';

	for(comma = 1;coor2[comma] != ','; comma++);
	ss << coor2.substr(1,comma-1) << ' ';
	ss << coor2.substr(comma+1,coor2.length()-comma-2) << ' ';

	ss >> temp_coords->x1 >> temp_coords->y1 >> temp_coords->x2 >> temp_coords->y2;

	//cout << temp_coords->x1 << " " << temp_coords->y1 << " " << temp_coords->x2 << " " << temp_coords->y2<<endl;

	return temp_coords;
}

Via* Manager::Parsing_via(string coor1){ // ex: (8,523)

	int comma;
	stringstream ss;
	Via *temp_via = new Via;
	for(comma = 1;coor1[comma] != ','; comma++);
	ss << coor1.substr(1,comma-1) << ' ';
	ss << coor1.substr(comma+1,coor1.length()-comma-2) << ' ';

	ss >> temp_via->x >> temp_via->y;

	//cout << temp_via->x << " "<< temp_via->y << endl;

	return temp_via;
}