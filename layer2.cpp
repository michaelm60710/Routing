#include "layer.h"
#include <iostream>
#include <algorithm>
#include <fstream>

using namespace std;

bool
sort_linex(GraphPoint* G1, GraphPoint* G2){
    return G1->x < G2->x;
}

void Layer::SpanningGraphConstruct_2(){
    //###1. Init Bound
    bound_map.clear();
    BoundLine_info* b_down  = new BoundLine_info(Width, 0, UP, Spacing-1, min_x, NULL);
    BoundLine_info* b_down1 = new BoundLine_info(Width, 0, DOWN, -1, min_x, NULL);//
    BoundLine_info* b_upper = new BoundLine_info(Width, 0, UP, Height+1, min_x, NULL);//
    BoundLine_info* b_upper1= new BoundLine_info(Width, 0, DOWN, Height+1-Spacing, min_x, NULL);
    bound_map.insert(pair< int , BoundLine_info*>(Spacing-1, b_down) );
    bound_map.insert(pair< int , BoundLine_info*>(-1, b_down1) );
    bound_map.insert(pair< int , BoundLine_info*>(Height+1, b_upper) );
    bound_map.insert(pair< int , BoundLine_info*>(Height+1-Spacing, b_upper1) );

}


GraphPoint* Layer::SGconstruct_2(Line *LLine, GraphPoint *GP_last, bool L_or_GP){ //L_or_GP=true -> LLine
    map< int , BoundLine_info* , less<int> >::iterator it1,it2, low_it,traverse_it;
    pair<map< int , BoundLine_info* , less<int> >::iterator,bool> status1,status2;
    bool p1,p2;
    int t_shape_type = 0;
    int temp_x = 0, temp_max_x = 0, temp_bound_x = 0, temp_y1 = 0, temp_y2 = 0;
    GraphPoint *GP1 = NULL, *GP2 = NULL, *GP_rshape = NULL;
    BoundLine_info *temp_bline;
    Cluster *temp_clu = NULL;

    /*
                    |
              5     |
        ----------- p1 ---------------
              3     |                |
     del 4          |  L (shape)     |              
              2     |                |
        ------------p2 ---------------
              1     |
                    |

    */
    if(!L_or_GP){ // gp_vec 
        // ### Init
        temp_x = temp_max_x = GP_last->x;
        temp_y1 = temp_y2 = GP_last->y;
        temp_clu = NULL;
        t_shape_type = OBSTACLE;
        // ### Init over, start (like Via)
        p1 = true;
        GP1 = NULL;
        BoundLine_info* b1 = new BoundLine_info(temp_max_x, temp_x, VIA, temp_y1, min_x, NULL);
        status1 = bound_map.insert(pair< int , BoundLine_info*>(temp_y1, b1) );

        it1 = status1.first;
        
        //############check point whether need to insert
        traverse_it = it1;
        ++traverse_it;
        if(status1.second==false && it1->second->max_x > temp_x ){//same y 
        	
            if(it1->second->Gp && it1->second->Gp->Shape_type==RSHAPE){//with Rshape connect
                GP_last->Add_edge(it1->second->Gp, temp_x, temp_y1, temp_x, temp_y1, Layer_pos, 0);
            }
            if(traverse_it->second->down_edge_GP && traverse_it->second->down_edge_GP->Shape_type==RSHAPE && traverse_it->second->Get_down_edge_x() >= temp_x){
                GP_last->Add_edge(traverse_it->second->down_edge_GP, temp_x, temp_y1, temp_x, temp_y1, Layer_pos, 0);
            }
        } 
        else if(status1.second!=false && traverse_it!=bound_map.end() && traverse_it->second->Get_down_edge_x() >= temp_x){
        	
            if(traverse_it->second->down_edge_GP && traverse_it->second->down_edge_GP->Shape_type==RSHAPE){
                GP_last->Add_edge(traverse_it->second->down_edge_GP, temp_x, temp_y1, temp_x, temp_y1, Layer_pos, 0);
            }
            p1 = false;//this point need to delete
        }
        else{
        	
            GP1 = GP_last;
            //5 construct edge
            temp_bound_x = min_x;
            traverse_it = it1;
            if(status1.second==false){ //it1->second->max_x <= temp_x
                temp_bline = traverse_it->second;
                if(temp_bline->Gp!=NULL && temp_bline->point_x > temp_bound_x ){
                	if(temp_bline->Gp->Shape_type!=OBSTACLE)
                		GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_bline->max_x, temp_y1, Layer_pos, 0);
                    else{
                    	GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                	}
                }
                if(temp_bline->max_x > temp_bound_x) temp_bound_x = temp_bline->max_x;
            }
            ++traverse_it;
            if(traverse_it->second->Gp!=NULL && traverse_it->second->max_x == temp_x )//test
            	GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, temp_x, traverse_it->second->point_y, Layer_pos, 0);//test

            if(traverse_it!=bound_map.end() && traverse_it->second->Get_down_edge_x() <= temp_x){
            	if(traverse_it->second->down_edge_GP !=NULL)//horizental
            		GP1->Add_edge(traverse_it->second->down_edge_GP, temp_x, temp_y1, traverse_it->second->Get_down_edge_x(), temp_y1, Layer_pos, 0);
				if(traverse_it->second->Get_down_edge_x() > temp_bound_x) temp_bound_x = traverse_it->second->Get_down_edge_x();
				
				temp_bline = traverse_it->second;
				if(temp_bline->Gp!=NULL && temp_bline->max_x >= temp_bound_x ){
					if(temp_bline->Gp->Shape_type!=OBSTACLE && temp_bline->max_x >= temp_x)//make it "vertical"
                		GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_x, temp_bline->point_y, Layer_pos, 0);
                	else
                    	GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                }   
                ++traverse_it;   	
            }

            
            for(;traverse_it!=bound_map.end();++traverse_it){
                temp_bline = traverse_it->second;
                if(temp_bound_x > temp_x) break;
                if(temp_bline->Gp!=NULL && temp_bline->point_x >= temp_bound_x ){
                	if(temp_bline->Gp->Shape_type==RSHAPE && temp_bline->max_x >= temp_x)//make it "vertical"
                		GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_x, temp_bline->point_y, Layer_pos, 0);
                	else
                    	GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_bline->max_x, temp_bline->point_y, Layer_pos, 0);
                    if(temp_bline->max_x == temp_x) break;
                }
                if(temp_bline->max_x > temp_bound_x) temp_bound_x = temp_bline->max_x;
            }
            //1
            temp_bound_x = min_x;
            traverse_it = it1;
            
            if(it1!=bound_map.begin()) {
            	--traverse_it;
            	//test
            	temp_bline = traverse_it->second;
            	if(temp_bline->Gp!=NULL && temp_bline->max_x == temp_x )
            		GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_x, temp_bline->point_y, Layer_pos, 0);
            }
            while(1){
                temp_bline = traverse_it->second;
                if(temp_bound_x > temp_x) break;
                if(temp_bline->Gp!=NULL && temp_bline->point_x >= temp_bound_x ){
                	if(temp_bline->Gp->Shape_type==RSHAPE && temp_bline->max_x >= temp_x)//make it "vertical"
                		GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_x, temp_bline->point_y, Layer_pos, 0);
                	else
                    	GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_bline->max_x, temp_bline->point_y, Layer_pos, 0);
                    if(temp_bline->max_x == temp_x) break;
                }
                if(temp_bline->max_x > temp_bound_x) temp_bound_x = temp_bline->max_x;
                if(traverse_it==bound_map.begin()) break;
                --traverse_it;
            }

            it1->second->Gp = GP1;

        }
        
        if(status1.second==false){
            if(it1->second->max_x <= temp_x)
                it1->second->point_x = temp_x;
            if(it1->second->max_x <= temp_max_x)
                it1->second->max_x = temp_max_x;
        }

        if(p1==false) bound_map.erase(it1);
        else{
            traverse_it = it1;
            ++traverse_it;
            if(traverse_it!=bound_map.end()) 
               it1->second->Change_up_edge(traverse_it->second->down_edge_GP, traverse_it->second->Get_down_edge_x()); 
            if(it1!=bound_map.begin()) {
                traverse_it = it1;
                --traverse_it;
                it1->second->Change_down_edge(traverse_it->second->up_edge_GP, traverse_it->second->Get_up_edge_x());
            }
        }

        return GP_last;
    }
    else if(L_or_GP){
        temp_x = LLine->x;
        temp_max_x = LLine->S->coords->x2;
        temp_clu = LLine->S->clu;
        temp_y1 = LLine->y+LLine->length;
        temp_y2 = LLine->y;
        t_shape_type = LLine->S->Shape_type; 
    }
    

    //for up down x edge GP
    if(t_shape_type==RSHAPE) GP_rshape = temp_clu->Add_GP(LLine, UP, G_point_num);
    else                     GP_rshape = NULL;

    //start
    if(t_shape_type==VIA){
        p1 = true;
        GP1 = NULL;
        BoundLine_info* b1 = new BoundLine_info(temp_max_x, temp_x, VIA, temp_y1, min_x, GP_rshape);
        status1 = bound_map.insert(pair< int , BoundLine_info*>(temp_y1, b1) );

        it1 = status1.first;
        
        //############check point whether need to insert
        traverse_it = it1;
        ++traverse_it;

        if(status1.second==false && it1->second->max_x > temp_x ){ //same y 
            //up point no need 
        }
        else if(status1.second!=false && traverse_it!=bound_map.end() && traverse_it->second->Get_down_edge_x() >= temp_x){
            //up point no need 
            if( traverse_it->second->Get_down_edge_x() >= temp_max_x) p1 = false;//this point need to delete
        }
        else{
            GP1 = temp_clu->Add_GP(LLine, UP, G_point_num);
            int temp_bound_x_2 = min_x;
            //1 and 5 construct edge
            temp_bound_x = min_x;
            traverse_it = it1;
            if(status1.second==false){
                temp_bline = traverse_it->second;
                if(temp_bline->Gp!=NULL && temp_bline->point_x > temp_bound_x ){
                    GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                }
                if(temp_bline->max_x > temp_bound_x) temp_bound_x = temp_bound_x_2 = temp_bline->max_x;
            }
            ++traverse_it;
            for(;traverse_it!=bound_map.end();++traverse_it){
                temp_bline = traverse_it->second;
                if(temp_bound_x > temp_x) break;
                if(temp_bline->Gp!=NULL && temp_bline->point_x > temp_bound_x ){
                    GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);

                }
                if(temp_bline->max_x > temp_bound_x) temp_bound_x = temp_bline->max_x;
            }

            //1
            temp_bound_x = temp_bound_x_2;
            traverse_it = it1;
            
            if(it1!=bound_map.begin()) --traverse_it;
            while(1){
                temp_bline = traverse_it->second;
                if(temp_bound_x > temp_x) break;
                if(temp_bline->Gp!=NULL && temp_bline->point_x > temp_bound_x ){
                    GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                }
                if(temp_bline->max_x > temp_bound_x) temp_bound_x = temp_bline->max_x;
                if(traverse_it==bound_map.begin()) break;
                --traverse_it;
            }

            it1->second->Gp = GP1;//////

        }
        
        if(status1.second==false){
            if(it1->second->max_x <= temp_x)
                it1->second->point_x = temp_x;
            if(it1->second->max_x <= temp_max_x)
                it1->second->max_x = temp_max_x;
        }

        if(p1==false) bound_map.erase(it1);
        else{
            traverse_it = it1;
            ++traverse_it;
            if(traverse_it!=bound_map.end()) 
               it1->second->Change_up_edge(traverse_it->second->down_edge_GP, traverse_it->second->Get_down_edge_x()); // it1->second->up_edge_x = traverse_it->second->Get_down_edge_x();
            if(it1!=bound_map.begin()) {
                traverse_it = it1;
                --traverse_it;
                it1->second->Change_down_edge(traverse_it->second->up_edge_GP, traverse_it->second->Get_up_edge_x());//it1->second->down_edge_x = traverse_it->second->Get_up_edge_x();
            }
        }
    }
    else if(LLine->LR==LEFT){ //LEFT
        p1 = p2 = true;
        GP1 = GP2 = NULL;
        BoundLine_info* b1 = new BoundLine_info(temp_max_x, temp_x, UP, temp_y1, min_x, GP_rshape);
        BoundLine_info* b2 = new BoundLine_info(temp_max_x, temp_x, DOWN, temp_y2, min_x, GP_rshape);

        status1 = bound_map.insert(pair< int , BoundLine_info*>(temp_y1, b1) );
        it1 = status1.first;
        status2 = bound_map.insert(pair< int , BoundLine_info*>(temp_y2, b2) );
        it2 = status2.first;
        
        //############check it2 whether need to insert
        traverse_it = it2;
        if(traverse_it!=bound_map.begin()) --traverse_it;

        if(status2.second==false && it2->second->max_x > temp_x ); //same y 
        else if(status2.second!=false && it2!=bound_map.begin() && traverse_it->second->Get_up_edge_x() >= temp_x){
            //Down point no need 
            if(traverse_it->second->Get_up_edge_x() >= temp_max_x) p2 = false;//dont inert this x line 
        }
        else{
            GP2 = temp_clu->Add_GP(LLine, DOWN, G_point_num);
            if(t_shape_type==RSHAPE){//0726
              //1 and 2 construct edge
                temp_bound_x = min_x;
                traverse_it = it2;
                if(status2.second==false){
                    if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                        GP2->Add_edge(traverse_it->second->Gp, temp_x, temp_y2, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, 0);
                    }
                    if(traverse_it->second->max_x == temp_x); //trivial cases.... = =
                    else if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                }
                if(it2!=bound_map.begin()) --traverse_it;
                if(temp_bound_x <= temp_x){
                    while(1){
                        temp_bline = traverse_it->second;
                        if(temp_bline->Gp!=NULL && temp_bline->point_x > temp_bound_x ){
                            GP2->Add_edge(temp_bline->Gp, temp_x, temp_y2, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                        }
                        if(temp_bline->max_x > temp_bound_x) {
                            temp_bound_x = temp_bline->max_x;
                            if(temp_bound_x > temp_x) break;
                        }
                        if(traverse_it==bound_map.begin()) break;
                        --traverse_it;
                    }
                }
                if(traverse_it->second->Gp!=NULL && traverse_it->second->Gp->Shape_type==RSHAPE && traverse_it->second->max_x >= temp_x)//vertical 
                    GP2->Add_edge(traverse_it->second->Gp, temp_x, temp_y2, temp_x, traverse_it->second->point_y, Layer_pos, 0);
                //2
                if(status2.second==false) temp_bound_x = it2->second->max_x;
                else                      temp_bound_x = min_x;
                traverse_it = it2;
                ++traverse_it;
                if(it2->first!=it1->first){//if the shape is not a wire
                    if(traverse_it->second->down_edge_GP !=NULL && traverse_it->second->Get_down_edge_x() <= temp_x)//bug : it1 = it2 + 2 && RSHAPE
                        GP2->Add_edge(traverse_it->second->down_edge_GP, temp_x, temp_y2, traverse_it->second->Get_down_edge_x(), temp_y2, Layer_pos, 0);

                    if(t_shape_type!=RSHAPE ){
                        for(;traverse_it!=it1;++traverse_it){
                            temp_bline = traverse_it->second;
                            if(temp_bound_x >= temp_x) break;
                            if(temp_bline->Gp!=NULL && temp_bline->point_x > temp_bound_x ){
                                GP2->Add_edge(temp_bline->Gp, temp_x, temp_y2, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                            }
                            if(temp_bline->max_x > temp_bound_x) temp_bound_x = temp_bline->max_x;
                        }
                    }
                    else{ //RSHAPE
                        for(;traverse_it!=it1;++traverse_it){
                            temp_bline = traverse_it->second;
                            if(temp_bline->Gp!=NULL && temp_bline->point_x < temp_x ){
                                GP2->Add_edge(temp_bline->Gp, temp_x, temp_bline->point_y, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                            }
                        }                   
                    }
                    
                }  
                it2->second->Gp = GP2;
            } 
            else it2->second->Gp = NULL;

        }

        //############check it1  whether need to insert
        traverse_it = it1;
        ++traverse_it;

        if(status1.second==false && it1->second->max_x > temp_x ); //same y 
        else if(status1.second!=false && traverse_it!=bound_map.end() && traverse_it->second->Get_down_edge_x() >= temp_x){
            //up point no need 
            if( traverse_it->second->Get_down_edge_x() >= temp_max_x) p1 = false;//this point need to delete
        }
        else{
            GP1 = temp_clu->Add_GP(LLine, UP, G_point_num);
            if(t_shape_type==RSHAPE){
                // 5 and 3 construct edge
                if(status1.second==false && it1->second->max_x != temp_x) temp_bound_x = it1->second->max_x;//trivial cases
                else                                                      temp_bound_x = min_x;
                traverse_it = it1;
                ++traverse_it;
                for(;traverse_it!=bound_map.end();++traverse_it){
                    temp_bline = traverse_it->second;
                    if(temp_bline->Gp!=NULL && temp_bline->point_x > temp_bound_x ){
                        GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                    }
                    if(temp_bline->max_x > temp_bound_x) {
                        temp_bound_x = temp_bline->max_x;
                        if(temp_bound_x > temp_x) {
                            if(temp_bline->Gp!=NULL && temp_bline->Gp->Shape_type==RSHAPE && temp_bline->max_x >= temp_x)//vertical 
                                GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_x, temp_bline->point_y, Layer_pos, 0);   
                            break;
                        }
                    }
                }
                
                //3
                temp_bound_x = min_x;
                traverse_it = it1;
                if(status1.second==false){
                    if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                        GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, 0);
                    }
                    if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                }

                --traverse_it;
                if(it2->first!=it1->first){//if the shape is not a wire
                    if(traverse_it->second->up_edge_GP !=NULL && traverse_it->second->Get_up_edge_x() <= temp_x)
                        GP1->Add_edge(traverse_it->second->up_edge_GP, temp_x, temp_y1, traverse_it->second->Get_up_edge_x(), temp_y1, Layer_pos, 0);

                    if(t_shape_type!=RSHAPE ){

                        for(;traverse_it!=it2;--traverse_it){
                            temp_bline = traverse_it->second;
                            if(temp_bound_x >= temp_x) break;
                            if(temp_bline->Gp!=NULL && temp_bline->point_x > temp_bound_x ){
                                GP1->Add_edge(temp_bline->Gp, temp_x, temp_y1, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                            }
                            if(temp_bline->max_x > temp_bound_x) temp_bound_x = temp_bline->max_x;
                        }
                    }
                    else if(GP2==NULL){
                        for(;traverse_it!=it2;--traverse_it){
                            temp_bline = traverse_it->second;
                            if(temp_bline->Gp!=NULL && temp_bline->point_x < temp_x ){
                                GP1->Add_edge(temp_bline->Gp, temp_x, temp_bline->point_y, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                            }
                        }
                    }
                }

                //trivial cases: itr_gp1-1 = itr_gp2 
                traverse_it = it1;
                --traverse_it;
                if(traverse_it==it2){
                    traverse_it = it1;
                    ++traverse_it;
                    if(traverse_it!=bound_map.end() && traverse_it->second->down_edge_GP !=NULL && traverse_it->second->Get_down_edge_x() <= temp_x){
                        GP1->Add_edge(traverse_it->second->down_edge_GP, temp_x, temp_y1, traverse_it->second->Get_down_edge_x(), temp_y1, Layer_pos, 0);
                    }

                }
                it1->second->Gp = GP1;//0726
            } 
            else  it1->second->Gp = NULL;

        }
        
        //############delete mid (refrech bound info)
        traverse_it = it2;
        ++traverse_it;
        if(it2->first!=it1->first)//if the shape is not a wire
            for(;traverse_it!=it1; ++traverse_it){
                temp_bline = traverse_it->second;
                if(temp_bline->max_x <= temp_max_x ) {
                	if(temp_bline->Gp != NULL && temp_bline->point_x >= temp_x){//gp point
                		//if(GP1 != NULL )  GP1->Add_edge(temp_bline->Gp, temp_bline->point_x, temp_bline->point_y, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                		//if(GP2 != NULL )  GP2->Add_edge(temp_bline->Gp, temp_bline->point_x, temp_bline->point_y, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                	}
                    delete temp_bline;
                    bound_map.erase(traverse_it++);
                    --traverse_it;
                }
                else{
                    if(temp_bline->Get_up_edge_x() < temp_max_x)   temp_bline->Change_up_edge(GP_rshape, temp_max_x);
                    if(temp_bline->Get_down_edge_x() < temp_max_x) temp_bline->Change_down_edge(GP_rshape, temp_max_x);
                    temp_bline->Gp = NULL;
                }
            }
        
        if(status1.second==false){
            temp_bline = it1->second;
            if(temp_bline->Get_down_edge_x() < temp_max_x) temp_bline->Change_down_edge(GP_rshape, temp_max_x);//temp_bline->down_edge_x = temp_max_x;
            delete b1;
            if(temp_bline->max_x <= temp_x)
                temp_bline->point_x = temp_x;
            if(temp_bline->max_x < temp_max_x)
                temp_bline->max_x = temp_max_x;
        }
        if(status2.second==false) {
            temp_bline = it2->second;
            if(temp_bline->Get_up_edge_x() < temp_max_x) temp_bline->Change_up_edge(GP_rshape, temp_max_x);//temp_bline->up_edge_x = temp_max_x;
            delete b2;
            if(temp_bline->max_x <= temp_x)
                temp_bline->point_x = temp_x;
            if(temp_bline->max_x < temp_max_x)
                temp_bline->max_x = temp_max_x;
        }

        //############
        if(p1==false) {
            delete it1->second;
            bound_map.erase(it1);
        }
        else{
            traverse_it = it1;
            ++traverse_it;
            if(traverse_it!=bound_map.end())
                it1->second->Change_up_edge(traverse_it->second->down_edge_GP, traverse_it->second->Get_down_edge_x());
        }
        if(p2==false) {
            delete it2->second;
            bound_map.erase(it2);
        }
        else{
            if(it2!=bound_map.begin()) {
                traverse_it = it2;
                --traverse_it;
                it2->second->Change_down_edge(traverse_it->second->up_edge_GP, traverse_it->second->Get_up_edge_x());
            }
        }
    }
    else{ //RIGHT
        p1 = p2 = false;

        it1 = bound_map.lower_bound(temp_y1);
        it2 = bound_map.lower_bound(temp_y2);
        //############check it2 whether exist
        for (traverse_it = it2; traverse_it!=bound_map.end();++traverse_it){
            if(traverse_it->first > temp_y2) break;
            if(traverse_it->first == temp_y2 && traverse_it->second->max_x == temp_x ){
                p2 = true;
                break;
            }
        }            
        if(p2 && t_shape_type!=OBSTACLE){
            GP2 = temp_clu->Add_GP(LLine, DOWN, G_point_num);
            //1 construct edge
            temp_bound_x =  LLine->S->coords->x1;
            traverse_it = it2;
            if(it2->second->Gp!=NULL && it2->second->Gp!=GP2) GP2->Add_edge(it2->second->Gp, temp_x, temp_y2, temp_x, temp_y2, Layer_pos, 0);//gp_point & rshape
            if(it2!=bound_map.begin()) --traverse_it;
            while(1){
                temp_bline = traverse_it->second;
                if(temp_bline->Gp!=NULL && temp_bline->point_x > temp_bound_x ){
                    GP2->Add_edge(temp_bline->Gp, temp_bline->point_x, temp_y2, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);

                }
                if(temp_bline->max_x > temp_bound_x) temp_bound_x = temp_bline->max_x;
                if(temp_bound_x > temp_x) break;
                if(traverse_it==bound_map.begin()) break;
                --traverse_it;
            }    

            it2->second->Gp = GP2;
            it2->second->point_x = temp_x; 
        }
        if(p2) it2->second->point_x = temp_x; 
        //############check it1  whether exist
        for (traverse_it = it1; traverse_it!=bound_map.end();++traverse_it){
            if(traverse_it->first > temp_y1) break;
            if(traverse_it->first == temp_y1 && traverse_it->second->max_x == temp_x ){
                p1 = true;
                break;
            }
        }            
        if(p1 && t_shape_type!=OBSTACLE){
            GP1 = temp_clu->Add_GP(LLine, UP, G_point_num);
            //5 construct edge
            temp_bound_x = LLine->S->coords->x1;
            traverse_it = it1;
            if(it1->second->Gp!=NULL && it1->second->Gp!=GP1) GP1->Add_edge(it1->second->Gp, temp_x, temp_y1, temp_x, temp_y1, Layer_pos, 0);//gp_point & rshape
            ++traverse_it;
            for(;traverse_it!=bound_map.end();++traverse_it){
                temp_bline = traverse_it->second;
                if(temp_bound_x > temp_x) break;
                if(temp_bline->Gp!=NULL && temp_bline->point_x > temp_bound_x ){
                    GP1->Add_edge(temp_bline->Gp, temp_bline->point_x, temp_y1, temp_bline->point_x, temp_bline->point_y, Layer_pos, 0);
                }
                if(temp_bline->max_x > temp_bound_x) temp_bound_x = temp_bline->max_x;
            }  

            it1->second->Gp = GP1;
            it1->second->point_x = temp_x;
        }
        if(p1) it1->second->point_x = temp_x;
    }
    return NULL;//GP1; // no use?
}

void Layer::SGcons_RshapeOverlap_2(GraphPoint *GP1){
	if(GP1==NULL) return;
	map< int , BoundLine_info* , less<int> >::iterator it1;
    int temp_x,temp_y1;	
    GraphPoint *GP_overlap = NULL;

	temp_x = GP1->x;
    temp_y1 = GP1->y;
    //### 1.find the overlapping
	it1 = bound_map.lower_bound(temp_y1);
	if( it1 != bound_map.end() ){
		if (it1->second->down_edge_GP!=NULL && it1->second->Get_down_edge_x() >= temp_x) {
			GP_overlap = it1->second->down_edge_GP;
			GP1->Add_edge(GP_overlap, temp_x, temp_y1, temp_x, temp_y1, Layer_pos, Via_cost);
		}
		if(it1->second->max_x >= temp_x && it1->second->point_y == temp_y1 && it1->second->Gp!=NULL){
			GP_overlap = it1->second->Gp;
			GP1->Add_edge(GP_overlap, temp_x, temp_y1, temp_x, temp_y1, Layer_pos, Via_cost);
		}
	}
}