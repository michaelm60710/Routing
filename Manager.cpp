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
	//cout << "SGC...\n";
	SpanningGraphConstruct();
	//Step 2
	//cout << "STC...\n";
	SpanningTreeConstruct();

	//Step 1.2
	//cout << "SGC2...\n";
	//Reconstruct();
	//SpanningGraphConstruct_2();
	//SpanningTreeConstruct_2();
	
	// plot test
    //for(int i =0;i<MetalLayers;i++) all_layer[i].check_point_svg(itos1(i));



    //OutputFile
    //Output_test(Output_file); // output all the edges and vias
    Output(Output_file);
    //Verify();

    //test
    cout << "all edge: " << (*(gp_list.begin()))->construct_edge << ", min edge: " << (*(gp_list.begin()))->construct_min_edge << endl;
}



void Manager::Parsing(const char* Input_file){
	cout << "==================" << Input_file << "================" << endl;
	string garbage,layer,coor1,coor2;
	int l,all=0;
	//test
	MaxRshapeEdge = 0;
	ifstream i_file;

	i_file.open(Input_file,ios::in);
	//### 1. read parameters
	i_file>>garbage>>garbage>>ViaCost;
	i_file>>garbage>>garbage>>Spacing;
	i_file>>garbage>>garbage>>coor1>>coor2;
	Boundary = Parsing_coordinate(coor1,coor2);
	//cout << "boundary: " << Boundary->x1 << " " << Boundary->y1 << ", " << Boundary->x2 << " " << Boundary->y2 <<endl;
	i_file>>garbage>>garbage>>MetalLayers;
	i_file>>garbage>>garbage>>RoutedShapes;
	i_file>>garbage>>garbage>>RoutedVias;
	i_file>>garbage>>garbage>>Obstacles;

	//###1.2 construct
	all_layer.resize(MetalLayers);
	all_shape.resize(RoutedShapes+RoutedVias*2+Obstacles);
	min_x = Spacing - 1;
	for(int i = 0;i<MetalLayers;i++) {
		all_layer[i].Spacing = Spacing;
		all_layer[i].min_x   = min_x;//Spacing - 1;
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
		if(abs(temp_shape->coords->x1 - temp_shape->coords->x2) > MaxRshapeEdge ) MaxRshapeEdge = abs(temp_shape->coords->x1 - temp_shape->coords->x2);
		if(abs(temp_shape->coords->y1 - temp_shape->coords->y2) > MaxRshapeEdge ) MaxRshapeEdge = abs(temp_shape->coords->y1 - temp_shape->coords->y2);
	}
	for(int i = 0;i<MetalLayers;i++) all_layer[i].Max_dis  = 2*(MaxRshapeEdge/9) + 400;// MaxRshapeEdge/3 + 400
	for(int i = 0;i<MetalLayers;i++) all_layer[i].Max_ob_dis  = all_layer[i].Max_dis*2/3 - 200;//all_layer[i].Max_dis/2;
	max_dis_for_extra_obs = all_layer[0].Max_dis;
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
		all_layer[l].Obstacle_list_append(temp_shape);
		all_shape[all++] = temp_shape;
	}


	//###3. print & check
	//cout << "\nViaCost: " << ViaCost << endl;
	//cout << "Spacing: " << Spacing << endl;
	//cout << "MetalLayers: " <<MetalLayers << endl;
	//cout << "RoutedShapes: " <<RoutedShapes << endl;
	//cout << "RoutedVias: " <<RoutedVias << endl;
	cout << "Max R Edge: " << MaxRshapeEdge << endl;
	cout << "Max limit length: " << all_layer[0].Max_dis << endl;
	//cout << "Obstacles: " <<Obstacles << endl << endl;
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

	list < GraphPoint* >::iterator gp_itr;
	list<Edge_info*>::iterator edge_itr;
	multimap< int , via_pos* >::iterator via_itr,via_itr2;
    int layer_pos, XX, YY, x1, x2, y1, y2;
    int COST = 0;
	
	ofstream ofile;
	ofile.open(Output_file,ios::out);
	//test
	vector <multimap< int , int > > via_checker;//test
	via_checker.resize(MetalLayers);
    //### 1. init select = false
    for(gp_itr = gp_list.begin();gp_itr != gp_list.end(); ++gp_itr) (*gp_itr)->select = false;

    //### 2. Greedy steiner point
    for(gp_itr = gp_list.begin();gp_itr != gp_list.end(); ++gp_itr){
    	layer_pos = (*gp_itr)->Layer_pos;
    	(*gp_itr)->select = true;
        for(edge_itr = (*gp_itr)->final_edge.begin();edge_itr!=(*gp_itr)->final_edge.end(); ++edge_itr){
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
			COST += (abs(x1 - x2) + abs(y1 - y2) );

        }
        //####VIA output
	    for(edge_itr = (*gp_itr)->final_edge.begin();edge_itr!=(*gp_itr)->final_edge.end(); ++edge_itr){
    		if ((*edge_itr)->layer!=layer_pos){
    			x1 = (*edge_itr)->point_x1;
        		y1 = (*edge_itr)->point_y1;

                if((*edge_itr)->layer > layer_pos ){
                	for(int i = (*edge_itr)->layer;i>layer_pos;i--){
                		via_checker[i].insert(pair<int, int>(x1, y1) );
                	}
                }
                else if((*edge_itr)->layer < layer_pos){
					for(int i = (*edge_itr)->layer;i<layer_pos;i++){
                		via_checker[i+1].insert(pair<int, int>(x1, y1) );
                	}
                }
			} 
    	}

    }

    //### 3. check via
    multimap< int , int >::iterator itr, itr1;
    int x;
    bool out;
    for(int i=0;i<MetalLayers;i++){
    	for(itr = via_checker[i].begin();itr!=via_checker[i].end();++itr){
    		itr1 = itr;
    		x = itr1->first;
    		++itr1;
    		out = true;
    		while(itr1!=via_checker[i].end()){
    			if(itr1->first!=x) break;
    			if(itr1->second==itr->second) out = false;
    			++itr1;
    		}
    		if(out) {
    			//"check Via input" // not good enough
    			list < Shape* >::iterator via_itr;
    			bool out2 = true;
    			for(via_itr = all_layer[i-1].Via_list.begin(); via_itr != all_layer[i-1].Via_list.end(); ++via_itr){
    				if((*via_itr)->coords->x1 == x &&  (*via_itr)->coords->y1 == itr->second){
    					out2 = false;
    					break;
    				}
    			}
    			//"check Via input" over
    			if(out2){
    				ofile << "Via V" << i << " ("<< x << "," << itr->second << ")" << endl;
    				COST += ViaCost;
    			}
    		}
    	}
    }
    cout << "Calculate COST = " << COST << endl;

}

void Manager::Output_test(const char *Output_file){

	MAP_GP_edge::iterator map_edge_itr;
	multimap< int , via_pos* >::iterator via_itr,via_itr2;
	list < GraphPoint* >::iterator gp_itr,begin_itr,end_itr;
    int layer_pos, XX, YY, x1, x2, y1, y2;
    int COST = 0;
	
	ofstream ofile;
	ofile.open(Output_file,ios::out);
	//test
	vector <multimap< int , int > > via_checker;//test
	via_checker.resize(MetalLayers);
    //### 1. init select = false
    for(size_t i = 0; i < all_cluster.size(); i++){
        begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        if(all_cluster[i]->GetShapeType()==RSHAPE || all_cluster[i]->GetShapeType()==VIA){
            (*begin_itr)->select = false;
        }
        else{
            for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
                (*gp_itr)->select = false;
            }
        }
    }

    //### 2. Greedy steiner point
    int bound_x2 = Boundary->x2;
    int bound_y2 = Boundary->y2;
    for(size_t i = 0; i < all_cluster.size(); i++){
        begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();

        for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
	    	layer_pos = (*gp_itr)->Layer_pos;
	    	(*gp_itr)->select = true;
	        for(map_edge_itr = (*gp_itr)->map_edge.begin();map_edge_itr!=(*gp_itr)->map_edge.end(); ++map_edge_itr){

	        	if(map_edge_itr->second->Gp->select) continue;

	            x1 = map_edge_itr->second->point_x1;
	            y1 = map_edge_itr->second->point_y1;
	            x2 = map_edge_itr->second->point_x2;
	            y2 = map_edge_itr->second->point_y2;
	            if(x1 < Spacing || y1 < Spacing || x2 < Spacing || y2 < Spacing || x1 > bound_x2 - Spacing || x2 > bound_x2 - Spacing || y1 > bound_y2 - Spacing || y2 > bound_y2 - Spacing) continue;
	            
	            XX = x2 - x1;
				YY = y2 - y1;

				//####Edge output
				if(XX==0 && YY==0);
				else if(XX!=0 && YY!=0){ // add steiner_point
					ofile << "V-line M" << map_edge_itr->second->layer + 1 << " ("<< x2 << "," << y2 << ") (" << x2 << "," << y1 << ")" << endl;
					ofile << "H-line M" << map_edge_itr->second->layer + 1 << " ("<< x1 << "," << y1 << ") (" << x2 << "," << y1 << ")" << endl;

				}
				else if(XX==0){ //it is already vertical or H
					ofile << "V-line M" << map_edge_itr->second->layer + 1 << " ("<< x1 << "," << y1 << ") (" << x2 << "," << y2 << ")" << endl;				
				}
				else{ //YY==0
					ofile << "H-line M" << map_edge_itr->second->layer + 1 << " ("<< x1 << "," << y1 << ") (" << x2 << "," << y2 << ")" << endl;
				}
				COST += (abs(x1 - x2) + abs(y1 - y2) );

	        }
	        //####VIA output
		    for(map_edge_itr = (*gp_itr)->map_edge.begin();map_edge_itr!=(*gp_itr)->map_edge.end(); ++map_edge_itr){
	    		if (map_edge_itr->second->layer!=layer_pos){
	    			x1 = map_edge_itr->second->point_x1;
	        		y1 = map_edge_itr->second->point_y1;
	        	    if(x1 < Spacing || y1 < Spacing || x1 > bound_x2 - Spacing || y1 > bound_y2 - Spacing) continue;

	                if(map_edge_itr->second->layer > layer_pos ){
	                	for(int i = map_edge_itr->second->layer;i>layer_pos;i--){
	                		via_checker[i].insert(pair<int, int>(x1, y1) );
	                	}
	                }
	                else if(map_edge_itr->second->layer < layer_pos){
						for(int i = map_edge_itr->second->layer;i<layer_pos;i++){
	                		via_checker[i+1].insert(pair<int, int>(x1, y1) );
	                	}
	                }
				} 
	    	}

        }
    }

    //### 3. check via
    multimap< int , int >::iterator itr, itr1;
    int x;
    bool out;
    for(int i=0;i<MetalLayers;i++){
    	for(itr = via_checker[i].begin();itr!=via_checker[i].end();++itr){
    		itr1 = itr;
    		x = itr1->first;
    		++itr1;
    		out = true;
    		while(itr1!=via_checker[i].end()){
    			if(itr1->first!=x) break;
    			if(itr1->second==itr->second) out = false;
    			++itr1;
    		}
    		if(out) {
    			//"check Via input" // not good enough
    			list < Shape* >::iterator via_itr;
    			bool out2 = true;
    			for(via_itr = all_layer[i-1].Via_list.begin(); via_itr != all_layer[i-1].Via_list.end(); ++via_itr){
    				if((*via_itr)->coords->x1 == x &&  (*via_itr)->coords->y1 == itr->second){
    					out2 = false;
    					break;
    				}
    			}
    			//"check Via input" over
    			if(out2){
    				ofile << "Via V" << i << " ("<< x << "," << itr->second << ")" << endl;
    				COST += ViaCost;
    			}
    		}
    	}
    }
    cout << "Calculate COST = " << COST << endl;

}

/*void Manager::Verify(){

	list < GraphPoint* >::iterator gp_itr;
	list<Edge_info*>::iterator edge_itr;
	multimap< int , via_pos* >::iterator via_itr,via_itr2;
    int layer_pos, XX, YY, x1, x2, y1, y2;
    int COST = 0;
	
	//test
	vector <multimap< int , int > > via_checker;//test
	via_checker.resize(MetalLayers);
    //### 1. init select = false
    for(gp_itr = gp_list.begin();gp_itr != gp_list.end(); ++gp_itr) (*gp_itr)->select = false;

    //### 2. Greedy steiner point
    for(gp_itr = gp_list.begin();gp_itr != gp_list.end(); ++gp_itr){
    	layer_pos = (*gp_itr)->Layer_pos;
    	(*gp_itr)->select = true;
        for(edge_itr = (*gp_itr)->final_edge.begin();edge_itr!=(*gp_itr)->final_edge.end(); ++edge_itr){

        	if((*edge_itr)->Gp->select) continue;

            x1 = (*edge_itr)->point_x1;
            y1 = (*edge_itr)->point_y1;
            x2 = (*edge_itr)->point_x2;
            y2 = (*edge_itr)->point_y2;
            
            XX = x2 - x1;
			YY = y2 - y1;

			COST += (abs(x1 - x2) + abs(y1 - y2) );

        }
        //####VIA output
	    for(edge_itr = (*gp_itr)->final_edge.begin();edge_itr!=(*gp_itr)->final_edge.end(); ++edge_itr){
    		if ((*edge_itr)->layer!=layer_pos){
    			x1 = (*edge_itr)->point_x1;
        		y1 = (*edge_itr)->point_y1;

                if((*edge_itr)->layer > layer_pos ){
                	for(int i = (*edge_itr)->layer;i>layer_pos;i--){
                		via_checker[i].insert(pair<int, int>(x1, y1) );
                	}
                }
                else if((*edge_itr)->layer < layer_pos){
					for(int i = (*edge_itr)->layer;i<layer_pos;i++){
                		via_checker[i+1].insert(pair<int, int>(x1, y1) );
                	}
                }
			} 
    	}

    }

    //### 3. check via
    multimap< int , int >::iterator itr, itr1;
    int x;
    bool out;
    for(int i=0;i<MetalLayers;i++){
    	for(itr = via_checker[i].begin();itr!=via_checker[i].end();++itr){
    		itr1 = itr;
    		x = itr1->first;
    		++itr1;
    		out = true;
    		while(itr1!=via_checker[i].end()){
    			if(itr1->first!=x) break;
    			if(itr1->second==itr->second) out = false;
    			++itr1;
    		}
    		if(out) {
    			//"check Via input" // not good enough
    			list < Shape* >::iterator via_itr;
    			bool out2 = true;
    			for(via_itr = all_layer[i-1].Via_list.begin(); via_itr != all_layer[i-1].Via_list.end(); ++via_itr){
    				if((*via_itr)->coords->x1 == x &&  (*via_itr)->coords->y1 == itr->second){
    					out2 = false;
    					break;
    				}
    			}
    			//"check Via input" over
    			if(out2){
    				//ofile << "Via V" << i << " ("<< x << "," << itr->second << ")" << endl;
    				COST += ViaCost;
    			}
    		}
    	}
    }
    cout << "Verify :Calculate COST = " << COST << endl;
}*/


void Manager::Reconstruct(){
	list < GraphPoint* >::iterator gp_itr,begin_itr,end_itr;
	list<Edge_info*>::iterator edge_itr;
    int layer_pos, XX, YY, x1, x2, y1, y2;
	
	//xy position check
	vector <multimap< int , int > > xy_checker;
	xy_checker.resize(MetalLayers);

	//### 1. init select = false
    for(gp_itr = gp_list.begin();gp_itr != gp_list.end(); ++gp_itr) (*gp_itr)->select = false;

    //### 2. insert xy_checker
    for(gp_itr = gp_list.begin();gp_itr != gp_list.end(); ++gp_itr){
    	layer_pos = (*gp_itr)->Layer_pos;
    	(*gp_itr)->select = true;
        for(edge_itr = (*gp_itr)->final_edge.begin();edge_itr!=(*gp_itr)->final_edge.end(); ++edge_itr){
			
        	if((*edge_itr)->Gp->select) continue;

            x1 = (*edge_itr)->point_x1;
            y1 = (*edge_itr)->point_y1;
            x2 = (*edge_itr)->point_x2;
            y2 = (*edge_itr)->point_y2;
            
            XX = x2 - x1;
			YY = y2 - y1;

			//####Edge output : layer
			if(XX==0 && YY==0){//(*edge_itr)->layer
				xy_checker[(*gp_itr)->Layer_pos].insert(pair<int, int>(x1,y1) );
				xy_checker[(*edge_itr)->Gp->Layer_pos].insert(pair<int, int>(x1,y1) );
			}
			else if(XX!=0 && YY!=0){ // add 2 steiner_point
				xy_checker[(*gp_itr)->Layer_pos].insert(pair<int, int>(x1,y1) );
				xy_checker[(*edge_itr)->Gp->Layer_pos].insert(pair<int, int>(x2,y2) );
				xy_checker[(*edge_itr)->layer].insert(pair<int, int>(x1,y1) );
				xy_checker[(*edge_itr)->layer].insert(pair<int, int>(x1,y2) );
				xy_checker[(*edge_itr)->layer].insert(pair<int, int>(x2,y1) );
				xy_checker[(*edge_itr)->layer].insert(pair<int, int>(x2,y2) );
			}
			else{ //it is already vertical or H
				xy_checker[(*gp_itr)->Layer_pos].insert(pair<int, int>(x1,y1) );
				xy_checker[(*edge_itr)->Gp->Layer_pos].insert(pair<int, int>(x2,y2) );
				xy_checker[(*edge_itr)->layer].insert(pair<int, int>(x1,y1) );
				xy_checker[(*edge_itr)->layer].insert(pair<int, int>(x2,y2) );
			}

        }
        //####VIA output
        if((*gp_itr)->Shape_type==OBSTACLE);
        else{//RSHAPE or VIA (add VIA)
        	for(edge_itr = (*gp_itr)->final_edge.begin();edge_itr!=(*gp_itr)->final_edge.end(); ++edge_itr){
        		if ((*edge_itr)->layer!=layer_pos){
        			x1 = (*edge_itr)->point_x1;
            		y1 = (*edge_itr)->point_y1;

	                if((*edge_itr)->layer > layer_pos ){
	                	for(int i = (*edge_itr)->layer;i>layer_pos;i--){
	                		xy_checker[i].insert(pair<int, int>(x1, y1));
	                	}
	                }
	                else if((*edge_itr)->layer < layer_pos){
						for(int i = (*edge_itr)->layer;i<layer_pos;i++){
							xy_checker[i+1].insert(pair<int, int>(x1, y1));
	                	}
	                }
				}
        	}

        }

    }

    //### 3. remove the point which have same xy & 
    multimap< int , int >::iterator itr, itr1;
    vector<GraphPoint* >::iterator vec_itr;
    int x = 0, gp_len = 0;
    bool out;
    for(int i=0;i<MetalLayers;i++){
    	itr = xy_checker[i].begin();
    	while(itr!=xy_checker[i].end()){
    		itr1 = itr;
    		x = itr1->first;
    		++itr1;
    		out = true;
    		while(itr1!=xy_checker[i].end()){
    			if(itr1->first!=x) break;
    			if(itr1->second==itr->second) {
    				out = false;
    				break;
    			}
    			++itr1;
    		}
    		if(!out) {//have same xy, need to be deleted
    			xy_checker[i].erase(itr1);
    		}
    		else
    			++itr;
    	}
    	gp_len += xy_checker[i].size();
    }
    //### 4.  construct gp_vec in each layer
    cout << gp_len << endl;
    gp_vec.resize(gp_len);
    vec_itr = gp_vec.begin();
    for(int i=0;i<MetalLayers;i++)
    	for(itr = xy_checker[i].begin();itr!=xy_checker[i].end();++itr){
    		*vec_itr = new GraphPoint(i , itr->first, itr->second, all_layer[i].G_point_num);
    		//test ( for plot)
    		all_layer[i].layer_gp_vec.push_back(*vec_itr);

    		++vec_itr;
    	}
    //### 5. remove all gp map_edge
    for(size_t i = 0; i < all_cluster.size(); i++){
        begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
        	(*gp_itr)->map_edge.clear();
        }

    }

}

