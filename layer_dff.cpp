#include "layer.h"
#include <iostream>
#include <algorithm>
#include <fstream>

using namespace std;
/*
#Heuristic 
Consider the net shape in different layer. Project the net shape (corner) and keep the infomation in dff_map
Runtime -> 120%
Cost -> 99%
*/
int dff_layer_via_cost(int , int , int , int);

void Layer::diff_map_update(Line* LLine){

    map< int , BoundLine_info* , less<int> >::iterator it1,it2,traverse_it;
    pair<map< int , BoundLine_info* , less<int> >::iterator,bool> status1,status2;
    bool p1,p2;
    int limit_y;
    int temp_x,temp_max_x, temp_y1,temp_y2, temp_bound_x,temp_bound_x_2;
    BoundLine_info *temp_bline;
    GraphPoint *GP1 = NULL, *GP2 = NULL;

    /*
                    |
                    |
        ----------- p1 ---------------
                    |                |
                    |  L (shape)     |              
                    |                |
        ------------p2 ---------------
                    |
                    |

    */

    temp_max_x = LLine->S->coords->x2;
    temp_x = LLine->S->coords->x1;
    temp_y1 = LLine->y+LLine->length;
    temp_y2 = LLine->y;

    //start
    if(LLine->S->Shape_type==VIA){/*{{{*/
        p1 = true;
        BoundLine_info* b1 = new BoundLine_info(temp_max_x, temp_max_x, VIA, temp_y1, min_x, NULL);
        status1 = dff_bound_map.insert(pair< int , BoundLine_info*>(temp_y1, b1) );

        it1 = status1.first;
        
        //############check point whether need to insert
        traverse_it = it1;
        ++traverse_it;

        if(status1.second==false && it1->second->max_x > temp_x ){ //same y
        }
        else if(status1.second!=false && traverse_it!=dff_bound_map.end() && traverse_it->second->Get_down_edge_x() >= temp_x){
            p1 = false;//this point need to delete
        }
        else{
            it1->second->Gp = NULL;
        }
        
        if(status1.second==false){
            if(it1->second->max_x <= temp_max_x){
                it1->second->point_x = temp_max_x;
                it1->second->max_x = temp_max_x;
            }
        }

        if(p1==false) dff_bound_map.erase(it1);
        else{/*{{{*/
            traverse_it = it1;
            ++traverse_it;
            if(traverse_it!=dff_bound_map.end()) 
               it1->second->Change_up_edge(traverse_it->second->down_edge_GP, traverse_it->second->Get_down_edge_x());
            if(it1!=dff_bound_map.begin()) {
                traverse_it = it1;
                --traverse_it;
                it1->second->Change_down_edge(traverse_it->second->up_edge_GP, traverse_it->second->Get_up_edge_x());
            }/*}}}*/
        }
    }/*}}}*/
    else if(LLine->LR==LEFT){ //LEFT/*{{{*/
        p1 = p2 = true;
        BoundLine_info* b1 = new BoundLine_info(temp_max_x, temp_max_x, UP, temp_y1, min_x, NULL);
        BoundLine_info* b2 = new BoundLine_info(temp_max_x, temp_max_x, DOWN, temp_y2, min_x, NULL);

        status1 = dff_bound_map.insert(pair< int , BoundLine_info*>(temp_y1, b1) );
        it1 = status1.first;
        status2 = dff_bound_map.insert(pair< int , BoundLine_info*>(temp_y2, b2) );
        it2 = status2.first;
        
        //############check it2 whether need to insert
        traverse_it = it2;
        if(traverse_it!=dff_bound_map.begin()) --traverse_it;

        if(status2.second==false && it2->second->max_x > temp_x ){ //same y //Down point no need 
        }
        else if(status2.second!=false && it2!=dff_bound_map.begin() && traverse_it->second->Get_up_edge_x() >= temp_x){ //Down point no need 
            if(traverse_it->second->Get_up_edge_x() >= temp_max_x) p2 = false;//dont inert this x line 
        }
        else if(LLine->S->Shape_type==RSHAPE){
            GP2 = LLine->S->clu->Add_GP(LLine, DOWN, G_point_num);
            //1 and 2 construct edge
            temp_bound_x = temp_bound_x_2 = min_x;
            traverse_it = it2;
            if(status2.second==false){
                if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                    GP2->Add_edge(traverse_it->second->Gp, temp_x, temp_y2, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost*abs(traverse_it->second->Gp->Layer_pos - GP2->Layer_pos));
                }
                if(traverse_it->second->max_x > temp_bound_x){
                    if(traverse_it->second->max_x==temp_x) {
                        temp_bound_x   = traverse_it->second->Get_down_edge_x();
                        temp_bound_x_2 = traverse_it->second->Get_up_edge_x();
                    }
                    else {
                        temp_bound_x = temp_bound_x_2 = traverse_it->second->max_x;
                    }
                }
            }
            if(it2!=dff_bound_map.begin()) {
                --traverse_it;
                if(traverse_it->second->Get_up_edge_x() > temp_bound_x) temp_bound_x = traverse_it->second->Get_up_edge_x();
            }
            if(temp_bound_x <= temp_x){
                limit_y = temp_y2 - Max_dis;
                mmin_edge.Init();
                while(1){
                    temp_bline = traverse_it->second;
                    if(temp_bline->Gp!=NULL && temp_bline->point_x >= temp_bound_x && temp_bline->Gp->Layer_pos != GP2->Layer_pos){
                        mmin_edge.update(GP2, temp_bline->Gp, temp_x, temp_y2, temp_bline->point_x, temp_bline->point_y, Layer_pos, Via_cost*abs(temp_bline->Gp->Layer_pos - GP2->Layer_pos));
                        //GP2->Add_edge(temp_bline->Gp, temp_x, temp_y2, temp_bline->point_x, temp_bline->point_y, Layer_pos, Via_cost*abs(temp_bline->Gp->Layer_pos - GP2->Layer_pos));
                    }
                    if(temp_bline->max_x > temp_bound_x) {
                        temp_bound_x = temp_bline->max_x;
                        if(temp_bound_x > temp_x) break;
                    }
                    if(traverse_it==dff_bound_map.begin()) break;
                    if(temp_bline->point_y < limit_y) break;
                    --traverse_it;
                }
                mmin_edge.Add_min_edge(GP2);
            }
            
            //2
            temp_bound_x = temp_bound_x_2;
            traverse_it = it2;
            ++traverse_it;
            if(it2->first!=it1->first){//if the shape is not a wire
                if(traverse_it!=dff_bound_map.end() && traverse_it->second->Get_down_edge_x() > temp_bound_x) temp_bound_x = traverse_it->second->Get_down_edge_x();
                mmin_edge.Init();
                for(;traverse_it!=it1;++traverse_it){
                    temp_bline = traverse_it->second;
                    if(temp_bline->Gp && temp_bline->point_x < temp_x && temp_bline->Gp->Layer_pos != GP2->Layer_pos){
                        mmin_edge.update(GP2, temp_bline->Gp, temp_x, temp_bline->point_y, temp_bline->point_x, temp_bline->point_y, Layer_pos, Via_cost*abs(temp_bline->Gp->Layer_pos - GP2->Layer_pos));
                        //GP2->Add_edge(temp_bline->Gp, temp_x, temp_bline->point_y, temp_bline->point_x, temp_bline->point_y, Layer_pos, Via_cost*abs(temp_bline->Gp->Layer_pos - GP2->Layer_pos));
                    }
                }  
                mmin_edge.Add_min_edge(GP2);
            }

            it2->second->Gp = NULL;
        }
        else{it2->second->Gp = NULL;}

        //############check it1  whether need to insert
        traverse_it = it1;
        ++traverse_it;

        if(status1.second==false && it1->second->max_x > temp_x ){ //same y   //up point no need 
        }
        else if(status1.second!=false && traverse_it!=dff_bound_map.end() && traverse_it->second->Get_down_edge_x() >= temp_x){//up point no need 
            if( traverse_it->second->Get_down_edge_x() >= temp_max_x) p1 = false;//this point need to delete
        }
        else if(LLine->S->Shape_type==RSHAPE){
            GP1 = LLine->S->clu->Add_GP(LLine, UP, G_point_num);
            
            // 5 and 3 construct edge
            traverse_it = it1;
            temp_bound_x = temp_bound_x_2 = min_x;
            if(status1.second==false){
                if(traverse_it->second->Gp && traverse_it->second->point_x > temp_bound_x ){
                    GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost*abs(traverse_it->second->Gp->Layer_pos - GP1->Layer_pos));
                }
                if(traverse_it->second->max_x > temp_bound_x){
                    if(traverse_it->second->max_x==temp_x) {
                        temp_bound_x   = traverse_it->second->Get_up_edge_x();
                        temp_bound_x_2 = traverse_it->second->Get_down_edge_x();
                    }
                    else {
                        temp_bound_x = temp_bound_x_2 = traverse_it->second->max_x;
                    }
                }
            }

            ++traverse_it;
            if(traverse_it!=bound_map.end() && traverse_it->second->Get_down_edge_x() > temp_bound_x) {
                temp_bound_x = traverse_it->second->Get_down_edge_x();
            }
            limit_y = temp_y1 + Max_dis;
            mmin_edge.Init();
            for(;traverse_it!=bound_map.end();++traverse_it){
                temp_bline = traverse_it->second;
                if(temp_bline->Gp!=NULL && temp_bline->point_x >= temp_bound_x && temp_bline->Gp->Layer_pos != GP1->Layer_pos){
                    mmin_edge.update(GP1, temp_bline->Gp, temp_x, temp_y1, temp_bline->point_x, temp_bline->point_y, Layer_pos, Via_cost*abs(temp_bline->Gp->Layer_pos - GP1->Layer_pos));
                    //GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_bline->point_x, temp_bline->point_y, Layer_pos, Via_cost*abs(temp_bline->Gp->Layer_pos - GP1->Layer_pos));
                }
                if(temp_bline->max_x > temp_bound_x) {
                    temp_bound_x = temp_bline->max_x;
                    if(temp_bound_x > temp_x) break;
                }
                if(temp_bline->point_y > limit_y) break;
            }
            mmin_edge.Add_min_edge(GP1);
            
            //3
            traverse_it = it1;
            --traverse_it;
            if(GP2==NULL && it2->first!=it1->first){//if the shape is not a wire
                temp_bound_x = temp_bound_x_2;
                if(traverse_it->second->Get_up_edge_x() > temp_bound_x) temp_bound_x = traverse_it->second->Get_up_edge_x();
                
                mmin_edge.Init();
                for(;traverse_it!=it2;--traverse_it){
                    temp_bline = traverse_it->second;
                    if(temp_bline->Gp!=NULL && temp_bline->point_x < temp_x && temp_bline->Gp->Layer_pos != GP1->Layer_pos){
                         mmin_edge.update(GP1, temp_bline->Gp, temp_x, temp_bline->point_y, temp_bline->point_x, temp_bline->point_y, Layer_pos,Via_cost*abs(temp_bline->Gp->Layer_pos - GP1->Layer_pos));
                        //GP1->Add_edge(temp_bline->Gp, temp_x, temp_bline->point_y, temp_bline->point_x, temp_bline->point_y, Layer_pos,Via_cost*abs(temp_bline->Gp->Layer_pos - GP1->Layer_pos));
                    }
                }   
                mmin_edge.Add_min_edge(GP1);
            }


            it1->second->Gp = NULL;
        }
        else{it1->second->Gp = NULL;}

        {
	        //############delete mid (refrech bound info)
	        traverse_it = it2;
	        ++traverse_it;
	        if(it2->first!=it1->first)//if the shape is not a wire
	            for(;traverse_it!=it1; ++traverse_it){
	                temp_bline = traverse_it->second;
	                if(temp_bline->max_x <= temp_max_x ) {
	                    delete temp_bline;
	                    dff_bound_map.erase(traverse_it++);
	                    --traverse_it;
	                }
	                else{
	                    if(temp_bline->Get_up_edge_x() < temp_max_x)   temp_bline->Change_up_edge(NULL, temp_max_x);
	                    if(temp_bline->Get_down_edge_x() < temp_max_x) temp_bline->Change_down_edge(NULL, temp_max_x);
	                    temp_bline->Gp = NULL;
	                }
	            }
	        
	        //############
	        if(status1.second==false){
	            temp_bline = it1->second;
	            if(temp_bline->Get_down_edge_x() < temp_max_x) temp_bline->Change_down_edge(NULL, temp_max_x);
	            delete b1;
	            if(temp_bline->max_x <= temp_x)
	                temp_bline->point_x = temp_x;
	            if(temp_bline->max_x < temp_max_x)
	                temp_bline->max_x = temp_max_x;
	        }
	        if(status2.second==false) {
	            temp_bline = it2->second;
	            if(temp_bline->Get_up_edge_x() < temp_max_x) temp_bline->Change_up_edge(NULL, temp_max_x);
	            delete b2;
	            if(temp_bline->max_x <= temp_x)
	                temp_bline->point_x = temp_x;
	            if(temp_bline->max_x < temp_max_x)
	                temp_bline->max_x = temp_max_x;
	        }

	        //############
	        if(p1==false) {
	            delete it1->second;
	            dff_bound_map.erase(it1);
	        }
	        else{
	            traverse_it = it1;
	            ++traverse_it;
	            if(traverse_it!=dff_bound_map.end())
	                it1->second->Change_up_edge(traverse_it->second->down_edge_GP, traverse_it->second->Get_down_edge_x());//it1->second->up_edge_x = traverse_it->second->Get_down_edge_x();
	        }
	        if(p2==false) {
	            delete it2->second;
	            dff_bound_map.erase(it2);
	        }
	        else{
	            if(it2!=dff_bound_map.begin()) {
	                traverse_it = it2;
	                --traverse_it;
	                it2->second->Change_down_edge(traverse_it->second->up_edge_GP, traverse_it->second->Get_up_edge_x());//it2->second->down_edge_x = traverse_it->second->Get_up_edge_x();
	            }
	        }
    	}
    }/*}}}*/
    
}

void Layer::diff_map_insert_rshape_point(GraphPoint *GP, const int _x, const int _y){/*{{{*/
	/* Insert a rshape point (corner) into dff_bound_map */
    if(GP!=NULL && GP->Shape_type==RSHAPE);
    else  return;

    map< int , BoundLine_info* , less<int> >::iterator it1,traverse_it;
    pair<map< int , BoundLine_info* , less<int> >::iterator,bool> status1;
    bool p1;
    int temp_x = _x;


    p1 = true;
    BoundLine_info* b1 = new BoundLine_info(_x, _x, VIA, _y, min_x, NULL);
    status1 = dff_bound_map.insert(pair< int , BoundLine_info*>(_y, b1) );

    it1 = status1.first;
    
    //############check point whether need to insert
    traverse_it = it1;
    ++traverse_it;

    if(status1.second==false && it1->second->max_x > temp_x ){ //same y
    }
    else if(status1.second!=false && traverse_it!=dff_bound_map.end() && traverse_it->second->Get_down_edge_x() >= temp_x){
        p1 = false;//this point need to delete
    }
    else{
        it1->second->Gp = GP;
    }
    if(status1.second==false){
        if(it1->second->max_x <= temp_x){
            it1->second->point_x = temp_x;
            it1->second->max_x = temp_x;
        }
    }

    if(p1==false) dff_bound_map.erase(it1);
    else{
        traverse_it = it1;
        ++traverse_it;
        if(traverse_it!=dff_bound_map.end()) 
           it1->second->Change_up_edge(traverse_it->second->down_edge_GP, traverse_it->second->Get_down_edge_x());
        if(it1!=dff_bound_map.begin()) {
            traverse_it = it1;
            --traverse_it;
            it1->second->Change_down_edge(traverse_it->second->up_edge_GP, traverse_it->second->Get_up_edge_x());
        }
    }
}/*}}}*/
int dff_layer_via_cost(int layer1, int layer2, int this_layer, int via_cost){
	return (abs(layer1 - this_layer) + abs(layer2 - this_layer) )*via_cost;

}
