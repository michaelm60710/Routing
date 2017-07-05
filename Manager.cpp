#include "Manager.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

int Layer::G_point_num = 0;//Static Initialize

Manager::Manager(const char* Input_file,const char* Output_file){

	//Parsing
	Parsing(Input_file);
	
	//Step 1
	SpanningGraphConstruct();
	//Step 2
	SpanningTreeConstruct();
	//Step 3
	Optimize1();

	// plot test
    //for(int i =0;i<MetalLayers;i++) all_layer[i].check_point_svg(itos1(i));



    //OutputFile
    Output(Output_file);

}



void Manager::Parsing(const char* Input_file){
	cout << "Read Data:" << endl;
	string garbage,layer,coor1,coor2;
	int l,all=0;
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
	all_shape.resize(RoutedShapes+RoutedVias*2+Obstacles);
	for(int i = 0;i<MetalLayers;i++) {
		all_layer[i].Spacing = Spacing;
		all_layer[i].min_x   = Spacing - 1;
		all_layer[i].Width   = Boundary->x2 - Boundary->x1;
		all_layer[i].Height  = Boundary->y2 - Boundary->y1;
		all_layer[i].Layer_pos = i;
		all_layer[i].Via_cost  = ViaCost;
	}

	//###2. read RoutedShape
	//cout << "read RoutedShape..." << endl;
	for(int i = 0; i < RoutedShapes;i++){
		i_file>>garbage>>layer>>coor1>>coor2;//ex: RoutedShape>> M1>> (6469,2552)>> (6504,2558)
		l = int(layer[1]) - 49; // M1 -> l = 0, M2 -> l = 1
		Shape *temp_shape = new Shape(RSHAPE, l);
		temp_shape->coords = Parsing_coordinate(coor1,coor2);
		all_layer[l].Rshape_list_append(temp_shape);
		all_shape[all++] = temp_shape;
	}

	//###3. read RoutedVias
	//cout << "read RoutedVias..." << endl;
	for(int i = 0; i < RoutedVias;i++){
		i_file>>garbage>>layer>>coor1;
		l = int(layer[1]) - 49; // V1 -> 0, V2 -> 1
		Shape *temp_shape = new Shape(VIA, l);
		temp_shape->coords = Parsing_via(coor1);
		all_layer[l].Via_list_append(temp_shape);
		all_shape[all++] = temp_shape;
		Shape *temp_shape2 = new Shape(VIA, l+1);
		temp_shape2->coords = temp_shape->coords;
		all_layer[l+1].Upper_Via_list_append(temp_shape2);
		all_shape[all++] = temp_shape2;
	}

	//###2. read Obstacles
	//cout << "read Obstaclese..." << endl;
	for(int i = 0; i < Obstacles;i++){
		i_file>>garbage>>layer>>coor1>>coor2;
		l = int(layer[1]) - 49; // M1 -> 0, M2 -> 1
		Shape *temp_shape = new Shape(OBSTACLE, l);
		temp_shape->coords = Parsing_coordinate(coor1,coor2);
		//Add Spacing
		/*temp_shape->coords->x1 -= Spacing;
		temp_shape->coords->x2 += Spacing;
		temp_shape->coords->y1 -= Spacing;
		temp_shape->coords->y2 += Spacing;*/
		
		all_layer[l].Obstacle_list_append(temp_shape);
		all_shape[all++] = temp_shape;
	}


	//###3. print & check
	cout << "\nViaCost: " << ViaCost << endl;
	cout << "Spacing: " << Spacing << endl;
	cout << "MetalLayers: " <<MetalLayers << endl;
	cout << "RoutedShapes: " <<RoutedShapes << endl;
	cout << "RoutedVias: " <<RoutedVias << endl;
	cout << "Obstacles: " <<Obstacles << endl << endl;
	/*for(int i=0;i<MetalLayers;i++){
		cout << "LAYER " << i << ":"<<endl;
		cout << "   Rshape size: " << all_layer[i].get_Rshape_num() << endl;
		cout << "   Via size: " << all_layer[i].get_Via_num() << endl;
		cout << "   Obstacle size: " << all_layer[i].get_Obstacle_num() << endl;
	}*/

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

	return temp_coords;
}

Coords* Manager::Parsing_via(string coor1){ // ex: (8,523)

	int comma;
	stringstream ss;
	Coords *temp_coords = new Coords;
	for(comma = 1;coor1[comma] != ','; comma++);
	ss << coor1.substr(1,comma-1) << ' ';
	ss << coor1.substr(comma+1,coor1.length()-comma-2) << ' ';

	ss >> temp_coords->x1 >> temp_coords->y1;
	temp_coords->x2 = temp_coords->x1;
	temp_coords->y2 = temp_coords->y1;

	return temp_coords;
}


void Manager::Output(const char *Output_file){

	list < GraphPoint* >::iterator gp_itr,begin_itr,end_itr;
	list<Edge_info*>::iterator edge_itr;
	multimap< int , via_pos* > via_map;
	multimap< int , via_pos* >::iterator via_itr,via_itr2;
   	bool via_up, via_down;
    int layer_pos, XX, YY, x1, x2, y1, y2;
	
	ofstream ofile;
	ofile.open(Output_file,ios::out);

    //### 1. init select = false
    for(gp_itr = gp_list.begin();gp_itr != gp_list.end(); ++gp_itr) (*gp_itr)->select = false;

    //### 2. Greedy steiner point
    for(gp_itr = gp_list.begin();gp_itr != gp_list.end(); ++gp_itr){
    	layer_pos = (*gp_itr)->Layer_pos;
    	(*gp_itr)->select = true;
    	via_up = via_down = false;
        for(edge_itr = (*gp_itr)->final_edge.begin();edge_itr!=(*gp_itr)->final_edge.end(); ++edge_itr){
        	//Via check
        	if ((*edge_itr)->layer!=layer_pos){
				if((*edge_itr)->Gp->Layer_pos==layer_pos+1) via_up = true;
                else if((*edge_itr)->Gp->Layer_pos==layer_pos-1) via_down = true;
			} 

        	if((*edge_itr)->Gp->select) continue;
            x1 = (*edge_itr)->point_x1;
            y1 = (*edge_itr)->point_y1;
            x2 = (*edge_itr)->point_x2;
            y2 = (*edge_itr)->point_y2;
            
            XX = x2 - x1;
			YY = y2 - y1;
			
			//####Edge output
			if(XX==0 && YY==0);
			else if(XX!=0 && YY!=0){ // add steiner_point
				ofile << "V-line M" << (*edge_itr)->layer + 1 << " ("<< x2 << "," << y2 << ") (" << x2 << "," << y1 << ")" << endl;
				ofile << "H-line M" << (*edge_itr)->layer + 1 << " ("<< x1 << "," << y1 << ") (" << x2 << "," << y1 << ")" << endl;

			}
			else if(XX==0){ //it is already vertical or H
				ofile << "V-line M" << (*edge_itr)->layer + 1 << " ("<< x1 << "," << y1 << ") (" << x2 << "," << y2 << ")" << endl;				
			}
			else{ //YY==0
				ofile << "H-line M" << (*edge_itr)->layer + 1 << " ("<< x1 << "," << y1 << ") (" << x2 << "," << y2 << ")" << endl;
			}

        }
        //####VIA output
        if((*gp_itr)->Shape_type==OBSTACLE){
        	if(via_up)  ofile << "Via V" << layer_pos + 1 << " ("<< (*gp_itr)->x << "," << (*gp_itr)->y << ")" << endl;
        	if(via_down)ofile << "Via V" << layer_pos     << " ("<< (*gp_itr)->x << "," << (*gp_itr)->y << ")" << endl;
        }
        else{//RSHAPE or VIA (remove same via)
        	via_map.clear();
        	for(edge_itr = (*gp_itr)->final_edge.begin();edge_itr!=(*gp_itr)->final_edge.end(); ++edge_itr){
        		if ((*edge_itr)->layer!=layer_pos){
        			x1 = (*edge_itr)->point_x1;
            		y1 = (*edge_itr)->point_y1;
					if((*edge_itr)->Gp->Layer_pos==layer_pos+1){
						via_pos *tmp_vc = new via_pos(x1, y1, layer_pos + 1, true);
						via_map.insert(pair<int, via_pos*>(tmp_vc->x, tmp_vc));
					}
	                else if((*edge_itr)->Gp->Layer_pos==layer_pos-1){
						via_pos *tmp_vc = new via_pos(x1, y1, layer_pos, false);
						via_map.insert(pair<int, via_pos*>(tmp_vc->x, tmp_vc));
	                }
				} 
        	}
        	for(via_itr = via_map.begin();via_itr != via_map.end(); ++via_itr){
        		bool out = true;
        		int temp_x = via_itr->second->x;
        		int temp_y = via_itr->second->y;
        		via_itr2 = via_itr;
        		++via_itr2;
        		for(;via_itr2 != via_map.end(); ++via_itr2){
        			if(via_itr2->second->x != temp_x) break;
        			if(via_itr2->second->y == temp_y && via_itr2->second->up_or_down==via_itr->second->up_or_down) {
        				out = false;
        				break;
        			}
        		}
        		if(out){
        			ofile << "Via V" << via_itr->second->via_layer << " ("<< temp_x << "," << temp_y << ")" << endl;
        		}
        	}



        }

    }

}
