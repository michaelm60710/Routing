#include "Manager.h"
#include <iostream>
#include <algorithm>

using namespace std;

bool
sort_linex(Line* L1, Line* L2){
    return L1->x < L2->x;
}

void Manager::SpanningGraphConstruct(){
	cout << "test" << endl;
    pair<GraphPoint*, GraphPoint*> GP_result;

	//###1. Initialize
	for(int i =0;i<MetalLayers;i++) all_layer[i].SpanningGraphConstruct();

	//###2. Sort all shape line
	int l_idx=0;
	all_line.resize((RoutedShapes+Obstacles)*2 + RoutedVias);

    for(size_t s = 0; s < all_shape.size(); s++){
        Line *l_l = new Line;
        Line *l_r = new Line;
        l_l->S = l_r->S = all_shape[s];
        l_r->LR = RIGHT;
        l_l->LR = LEFT;
        l_l->x = all_shape[s]->coords->x1;
        l_r->x = all_shape[s]->coords->x2;
        l_l->y = l_r->y = all_shape[s]->coords->y1;
        l_l->length = l_r->length = all_shape[s]->coords->y2 - l_l->y;
    	if(all_shape[s]->Shape_type==VIA) {
         	all_line[l_idx++]  = l_l;
    	}
        else{
        	all_line[l_idx]  = l_l;
        	all_line[l_idx + 1] = l_r;
        	l_idx += 2;
        }



    }
    sort(all_line.begin(), all_line.end(), sort_linex);

    //###3.
    for(size_t s = 0; s < all_line.size(); s++){
    	/*if(all_line[s]->S->layer_position==0){
    		GP_result = all_layer[all_line[s]->S->layer_position].SGconstruct(all_line[s]);
    	}
        else if(all_line[s]->S->layer_position==1){
            GP_result = all_layer[all_line[s]->S->layer_position].SGconstruct(all_line[s]);
        }
        else */if(all_line[s]->S->layer_position==2){
            cerr << s << " ";
            if(all_line[s]->S->Shape_type==VIA) continue;
            GP_result = all_layer[all_line[s]->S->layer_position].SGconstruct(all_line[s]);
        }
    }

    
    
    //ConvertToUndirectedG();
    // plot test
    all_layer[0].check_point_svg();
}