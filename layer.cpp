#include "layer.h"
#include <iostream>
#include <algorithm>
#include <fstream>

using namespace std;


Layer::Layer()
:Rshape_num(0),Obstacle_num(0),Via_num(0),Upper_Via_num(0),Layer_Shape_num(0)//,G_point_num(0) //initialize
{
	//cout << "QQ";
}

bool Sort_Shape(Shape* a, Shape* b){
	return a->coords->x1 < b->coords->x1;
}



void Layer::SpanningGraphConstruct(){

	//#1.0 construct all_shape_vec
	int pos = 0;
	Layer_Shape_num = Rshape_num+Obstacle_num+Via_num+Upper_Via_num;
	all_shape_vec.resize(Layer_Shape_num);
	for (it_shape it_s= Rshape_list.begin();it_s != Rshape_list.end();++it_s)
		all_shape_vec[pos++] = (*it_s); 
	for (it_shape it_s= Obstacle_list.begin();it_s != Obstacle_list.end();++it_s)
		all_shape_vec[pos++] = (*it_s); 
    for (it_shape it_s= Via_list.begin();it_s != Via_list.end();++it_s)
        all_shape_vec[pos++] = (*it_s);    
    for (it_shape it_s= Upper_Via_list.begin();it_s != Upper_Via_list.end();++it_s)
        all_shape_vec[pos++] = (*it_s);  	
	
	//#1.1 construct cluster
    clustering_shape();

    //#1.2 obstacle add spacing
    for (it_shape it_s= Obstacle_list.begin();it_s != Obstacle_list.end();++it_s){
    	(*it_s)->coords->x1 -= Spacing;
		(*it_s)->coords->x2 += Spacing;
		(*it_s)->coords->y1 -= Spacing;
		(*it_s)->coords->y2 += Spacing;
    }


    //#2 Init y_upper bound and y_lower bound
    /*
    ------------  b_upper
                |
    ------------  b_upper1
    |
    |
    ------------  b_down
                |
    ------------  b_down1
    */
    BoundLine_info* b_down  = new BoundLine_info(Width, 0, UP, Spacing-1, min_x);
    BoundLine_info* b_down1 = new BoundLine_info(Width, 0, DOWN, -1, min_x);//
    BoundLine_info* b_upper = new BoundLine_info(Width, 0, UP, Height+1, min_x);//
    BoundLine_info* b_upper1= new BoundLine_info(Width, 0, DOWN, Height+1-Spacing, min_x);
    bound_map.insert(pair< int , BoundLine_info*>(Spacing-1, b_down) );
    bound_map.insert(pair< int , BoundLine_info*>(-1, b_down1) );
    bound_map.insert(pair< int , BoundLine_info*>(Height+1, b_upper) );
    bound_map.insert(pair< int , BoundLine_info*>(Height+1-Spacing, b_upper1) );

}

bool
sort_func_x1(Shape* S1, Shape* S2){
	if(S1->coords->x1 < S2->coords->x1) return true;
	else if(S1->coords->x1 == S2->coords->x1) return (S1->coords->y1 < S2->coords->y1);
	else return false;
}

void
Layer::clustering_shape(){
	//cout<<"Clustering..."<<endl;
	for(size_t s = 0; s < all_shape_vec.size(); s++){
		all_shape_vec[s]->clu = NULL;
		sort_by_x1.push_back(all_shape_vec[s]);
	}
    sort(sort_by_x1.begin(), sort_by_x1.end(), sort_func_x1);
    
    for(size_t s = 0; s < sort_by_x1.size(); s++){
        if(sort_by_x1[s]->clu == NULL){
            Cluster* C = new Cluster(sort_by_x1[s]);
            sort_by_x1[s]->clu = C;
            build_clu(sort_by_x1[s], C, s+1);
            all_cluster.push_back(C);
        }
    }
    size_t count_O = 0;
    size_t count_S = 0;
    for(size_t i = 0; i < all_cluster.size(); i++){
    	Shape* S = all_cluster[i]->shape_list.front();
    	if(S->Shape_type == RSHAPE)count_S ++;
    	else count_O ++;
    }
    cout << "Rshape size: " <<  Rshape_list.size() << ", Via size: " << Via_list.size() << ", Obstacle size: " << Obstacle_list.size() << endl;
    cout<<"total cluster: "<<all_cluster.size();
    cout<<" ( # of Obstacle cluster = "<<count_O;
    cout<<", # of Rshape cluster = "<<count_S<<" )"<<endl;


}

void
Layer::build_clu(Shape* S, Cluster* C, int idx){
    for(size_t s = idx; s < sort_by_x1.size(); s++){
    	while(sort_by_x1[s]->coords->x2 < S->coords->x1)s++;
    	if(sort_by_x1[s]->coords->x1 > S->coords->x2) break;
    	if(sort_by_x1[s]->clu == NULL){
    		if(overlap(S, sort_by_x1[s])){
    			sort_by_x1[s]->clu = C;
    			C->shape_list.push_back(sort_by_x1[s]);
    			build_clu(sort_by_x1[s], C, idx);
    		}
    	}
        
    }
}

bool
Layer::overlap(Shape* S1, Shape* S2){
	if(S1->coords->y1 > S2->coords->y1){
        if( S2->coords->y2 >= S1->coords->y1 ) return true;
    }
    else{
    	if( S1->coords->y2 >= S2->coords->y1 ) return true;
    }
    return false;
}


bool
sort_line_x(Line* L1, Line* L2){
    return L1->x < L2->x;
}


pair<GraphPoint*, GraphPoint*> Layer::SGconstruct(Line* LLine){
    map< int , BoundLine_info* , less<int> >::iterator it1,it2, low_it,traverse_it;
    pair<map< int , BoundLine_info* , less<int> >::iterator,bool> status1,status2;
    bool p1,p2; //t_shape_type,
    int temp_x,temp_max_x, temp_bound_x,temp_y1,temp_y2;
    GraphPoint *GP1 = NULL, *GP2 = NULL;
    Cluster *temp_clu;

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

    temp_x = LLine->x;
    temp_max_x = LLine->S->coords->x2;
    temp_clu = LLine->S->clu;
    temp_y1 = LLine->y+LLine->length;
    temp_y2 = LLine->y;

    if(LLine->S->Shape_type==VIA){
        p1 = true;
        GP1 = NULL;
        BoundLine_info* b1 = new BoundLine_info(temp_max_x, temp_x, VIA, temp_y1, min_x);
        status1 = bound_map.insert(pair< int , BoundLine_info*>(temp_y1, b1) );

        it1 = status1.first;
        
        //############check point whether need to insert
        traverse_it = it1;
        ++traverse_it;

        if(status1.second==false && it1->second->max_x > temp_x ){ //same y 
            //up point no need 
        }
        else if(status1.second!=false && traverse_it!=bound_map.end() && traverse_it->second->down_edge_x >= temp_x){
            //up point no need 
            if( traverse_it->second->down_edge_x >= temp_max_x) p1 = false;//this point need to delete
        }
        else{
            GP1 = temp_clu->Add_GP(LLine, UP, G_point_num);
            //1 and 5 construct edge
            temp_bound_x = min_x;
            traverse_it = it1;
            ++traverse_it;
            for(;traverse_it!=bound_map.end();++traverse_it){
                if(temp_bound_x >= temp_x) break;
                if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                    GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, 0);

                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
            }

            //1
            temp_bound_x = min_x;
            traverse_it = it1;
            if(status1.second==false){
                if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                    GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, 0);
                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
            }
            if(it1!=bound_map.begin()) --traverse_it;
            while(1){
                if(temp_bound_x >= temp_x) break;
                if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                    GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, 0);
                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                if(traverse_it==bound_map.begin()) break;
                --traverse_it;
            }

            it1->second->Gp = GP1;//////

        }
        
        if(status1.second==false){
            if(it1->second->max_x < temp_x)
                it1->second->point_x = temp_x;
            if(it1->second->max_x<temp_max_x)
                it1->second->max_x = temp_max_x;
        }

        if(p1==false) bound_map.erase(it1);
        else{
            traverse_it = it1;
            ++traverse_it;
            if(traverse_it!=bound_map.end())
                it1->second->up_edge_x = traverse_it->second->down_edge_x;
            if(it1!=bound_map.begin()) {
                traverse_it = it1;
                --traverse_it;
                it1->second->down_edge_x = traverse_it->second->Get_up_edge_x();
            }
        }

    }
    else if(LLine->LR==LEFT){ //LEFT
        p1 = p2 = true;
        GP1 = GP2 = NULL;
        BoundLine_info* b1 = new BoundLine_info(temp_max_x, temp_x, UP, temp_y1, min_x);
        BoundLine_info* b2 = new BoundLine_info(temp_max_x, temp_x, DOWN, temp_y2, min_x);

        status1 = bound_map.insert(pair< int , BoundLine_info*>(temp_y1, b1) );
        it1 = status1.first;
        status2 = bound_map.insert(pair< int , BoundLine_info*>(temp_y2, b2) );
        it2 = status2.first;
        
        //############check it2 whether need to insert
        traverse_it = it2;
        if(traverse_it!=bound_map.begin()) --traverse_it;

        if(status2.second==false && it2->second->max_x > temp_x ){ //same y 
            //Down point no need 
        }
        else if(status2.second!=false && it2!=bound_map.begin() && traverse_it->second->Get_up_edge_x() >= temp_x){
            //Down point no need 
            if(traverse_it->second->Get_up_edge_x() >= temp_max_x) p2 = false;//dont inert this x line 
        }
        else{
            GP2 = temp_clu->Add_GP(LLine, DOWN, G_point_num);
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
            while(1){
                if(temp_bound_x >= temp_x) break;
                if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                    GP2->Add_edge(traverse_it->second->Gp, temp_x, temp_y2, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, 0);
                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                if(traverse_it==bound_map.begin()) break;
                --traverse_it;
            }
            //2
            if(status2.second==false) temp_bound_x = it2->second->max_x;
            else                      temp_bound_x = min_x;
            traverse_it = it2;
            ++traverse_it;
            if(it2->first!=it1->first)//if the shape is not a wire
                for(;traverse_it!=it1;++traverse_it){
                    if(temp_bound_x >= temp_x) break;
                    if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                        GP2->Add_edge(traverse_it->second->Gp, temp_x, temp_y2, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, 0);
                    }
                    if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                }

            it2->second->Gp = GP2;

        }

        //############check it1  whether need to insert
        traverse_it = it1;
        ++traverse_it;

        if(status1.second==false && it1->second->max_x > temp_x ){ //same y 
            //up point no need 
        }
        else if(status1.second!=false && traverse_it!=bound_map.end() && traverse_it->second->down_edge_x >= temp_x){
            //up point no need 
            if( traverse_it->second->down_edge_x >= temp_max_x) p1 = false;//this point need to delete
        }
        else{
            GP1 = temp_clu->Add_GP(LLine, UP, G_point_num);
            
            // 5 and 3 construct edge
            if(status1.second==false && it1->second->max_x != temp_x) temp_bound_x = it1->second->max_x;//trivial cases
            else                      								  temp_bound_x = min_x;
            traverse_it = it1;
            ++traverse_it;
            for(;traverse_it!=bound_map.end();++traverse_it){
                if(temp_bound_x >= temp_x) break;
                if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                    GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, 0);

                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
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
            if(it2->first!=it1->first)//if the shape is not a wire
                for(;traverse_it!=it2;--traverse_it){
                    if(temp_bound_x >= temp_x) break;
                    if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                        GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, 0);
                    }
                    if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                }

            it1->second->Gp = GP1;//////

        }

        //############Obstacle left 2 point connect
        bool connect = true;
        if(GP1!=NULL && GP2!=NULL && LLine->S->Shape_type!=RSHAPE){
            traverse_it = it2;
            ++traverse_it;
            for(;traverse_it!=it1; ++traverse_it){
                if(traverse_it->second->max_x > temp_x ) connect = false;
            }
            if(connect) GP1->Add_edge(GP2, temp_x, temp_y1, temp_x, temp_y2, Layer_pos, 0);
        }
        
        
        //############delete mid (refrech bound info)
        traverse_it = it2;
        ++traverse_it;
        if(it2->first!=it1->first)//if the shape is not a wire
            for(;traverse_it!=it1; ++traverse_it){

                if(traverse_it->second->max_x <= temp_max_x ) {
                	delete traverse_it->second;
                    bound_map.erase(traverse_it++);
                    --traverse_it;
                }
                else{
                    if(traverse_it->second->Get_up_edge_x() < temp_max_x)   traverse_it->second->up_edge_x   = temp_max_x;
                    if(traverse_it->second->down_edge_x < temp_max_x) traverse_it->second->down_edge_x = temp_max_x;
                    traverse_it->second->Gp = NULL;
                }
            }
        if(it1->second->down_edge_x < temp_max_x) it1->second->down_edge_x = temp_max_x;// && status1.second==false 
        if(it2->second->Get_up_edge_x() < temp_max_x) it2->second->up_edge_x = temp_max_x;// && status2.second==false
        
        //############ bug? >max_x <= temp_x
        if(status1.second==false){
        	delete b1;
            if(it1->second->max_x <= temp_x)
                it1->second->point_x = temp_x;
            if(it1->second->max_x < temp_max_x)
                it1->second->max_x = temp_max_x;
        }
        if(status2.second==false) {
        	delete b2;
            if(it2->second->max_x <= temp_x)
                it2->second->point_x = temp_x;
            if(it2->second->max_x < temp_max_x)
                it2->second->max_x = temp_max_x;
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
                it1->second->up_edge_x = traverse_it->second->down_edge_x;
        }
        if(p2==false) {
        	delete it2->second;
        	bound_map.erase(it2);
        }
        else{
            if(it2!=bound_map.begin()) {
                traverse_it = it2;
                --traverse_it;
                it2->second->down_edge_x = traverse_it->second->Get_up_edge_x();
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
        if(p2){
            GP2 = temp_clu->Add_GP(LLine, DOWN, G_point_num);
            //1 construct edge
            temp_bound_x =  LLine->S->coords->x1;
            traverse_it = it2;
            if(it2!=bound_map.begin()) --traverse_it;
            while(1){
                if(temp_bound_x > temp_x) break;
                if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                    GP2->Add_edge(traverse_it->second->Gp, temp_x, temp_y2, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, 0);

                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                if(traverse_it==bound_map.begin()) break;
                --traverse_it;
            }

            //construct Obstacle edge
            if(LLine->S->Shape_type!=RSHAPE && it2->second->point_x==LLine->S->coords->x1 && it2->second->Gp)
                GP2->Add_edge(it2->second->Gp, temp_x, temp_y2, it2->second->point_x, temp_y2, Layer_pos, 0);

            it2->second->Gp = GP2;
            it2->second->point_x = temp_x;
        }

        //############check it1  whether exist
        for (traverse_it = it1; traverse_it!=bound_map.end();++traverse_it){
            if(traverse_it->first > temp_y1) break;
            if(traverse_it->first == temp_y1 && traverse_it->second->max_x == temp_x ){
                p1 = true;
                break;
            }
        }            
        if(p1){
            GP1 = temp_clu->Add_GP(LLine, UP, G_point_num);
            //5 construct edge
            temp_bound_x = LLine->S->coords->x1;
            traverse_it = it1;
            ++traverse_it;
            for(;traverse_it!=bound_map.end();++traverse_it){
                if(temp_bound_x > temp_x) break;
                if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                    GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, 0);
                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
            }

            //construct Obstacle edge
            if(LLine->S->Shape_type!=RSHAPE && it1->second->point_x==LLine->S->coords->x1 && it1->second->Gp!=NULL)
                GP1->Add_edge(it1->second->Gp, temp_x, temp_y1, it1->second->point_x, temp_y1, Layer_pos, 0);

            it1->second->Gp = GP1;
            it1->second->point_x = temp_x;
        }

        //############Obstacle right 2 point connect
        bool connect = true;
        if(p1 && p2 && LLine->S->Shape_type!=RSHAPE){
            traverse_it = it2;
            ++traverse_it;
            for(;traverse_it!=it1; ++traverse_it){
                if(traverse_it->second->max_x >= temp_x ) connect = false;
            }
            if(connect) GP1->Add_edge(GP2, temp_x, temp_y1, temp_x, temp_y2, Layer_pos, 0);
        }


    }
 	
    return pair<GraphPoint*, GraphPoint*>(GP1, GP2);
}

void Layer::SGconstruct_search(Line* LLine, GraphPoint *GP1, GraphPoint *GP2){

    map< int , BoundLine_info* , less<int> >::iterator it1,it2, low_it,traverse_it;
    map< int , BoundLine_info* , less<int> >::reverse_iterator ritr;
    pair<map< int , BoundLine_info* , less<int> >::iterator,bool> status1,status2;
    GraphPoint *_GP;
    int temp_x, temp_bound_x,temp_y1,temp_y2;

    /*
                    |
              5     |
        ---------------------------- p1 (temp_x, temp_y1) UP
              3     |               |
     del 4          |  L (shape)    |              
              2     |               |
        ---------------------------- p2 (temp_x, temp_y2) DOWN
              1     |
                    |

    */
    temp_x = LLine->x;
    temp_y1 = LLine->y+LLine->length;
    temp_y2 = LLine->y;
    it1 = bound_map.lower_bound(temp_y1);
    it2 = bound_map.lower_bound(temp_y2);

    //### 1.find the overlapping
    if(LLine->S->Shape_type==RSHAPE){
    	_GP = LLine->S->clu->Add_GP(LLine, UP, G_point_num); 
    	//incomplete
    	//p2
    	if(it2 != bound_map.end() && it2->second->down_edge_x >= temp_x && it2->second->max_x == it2->second->down_edge_x && it2->second->Gp!=NULL && it2->second->Gp->Shape_type==RSHAPE){
            _GP->Add_edge(it2->second->Gp, temp_x, temp_y2, temp_x, temp_y2, Layer_pos, Via_cost);
    	}
        //p1
        if(it1 != bound_map.end() && it1->second->down_edge_x >= temp_x && it1->second->max_x == it1->second->down_edge_x && it1->second->Gp!=NULL && it1->second->Gp->Shape_type==RSHAPE){
            _GP->Add_edge(it1->second->Gp, temp_x, temp_y1, temp_x, temp_y1, Layer_pos, Via_cost);
        }

    }

    //### 2.Search  
    if((GP1==NULL && GP2==NULL) || LLine->S->Shape_type==VIA ) return;
    if(GP1==NULL || GP2==NULL || temp_y1==temp_y2){ //just one point
    	if(GP1==NULL) {
    		_GP = GP2;
    		temp_y1 = temp_y2;
    		it1 = it2;
    	}
    	else {
    		 _GP = GP1;
    	}

        //1 and 5 construct edge
        //5
        temp_bound_x = min_x;
        traverse_it = it1;
        for(;traverse_it!=bound_map.end();++traverse_it){
            if(temp_bound_x >= temp_x) break;
            if(traverse_it->second->Gp!=NULL && traverse_it->second->max_x <= temp_x && traverse_it->second->point_x > temp_bound_x ){
                _GP->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);

            }
            if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
        }

        //1
        temp_bound_x = min_x;
        if(it1!=bound_map.end()){//iterator -- 
            traverse_it = it1;
            while(1){
                if(temp_bound_x >= temp_x) break;
                if(traverse_it->second->Gp!=NULL && traverse_it->second->max_x <= temp_x && traverse_it->second->point_x > temp_bound_x ){
                    _GP->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);
                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                if(traverse_it==bound_map.begin()) break;
                --traverse_it;
            }
        }
        else{ //reverse iterator ++ 
            //cout << "1 point ritr";
            for(ritr = bound_map.rbegin();ritr!=bound_map.rend();++ritr){
                if(temp_bound_x >= temp_x) break;
                if(ritr->second->Gp!=NULL && ritr->second->point_x > temp_bound_x ){
                    _GP->Add_edge(ritr->second->Gp, temp_x, temp_y1, ritr->second->point_x, ritr->second->point_y, Layer_pos, Via_cost);
                }
                if(ritr->second->max_x > temp_bound_x) temp_bound_x = ritr->second->max_x;
            }
        }
        

        
    }
    else{ //bug: it1 it2 is lower bound
        //1 and 2 construct edge
        temp_bound_x = min_x;
        traverse_it = it2;
        if(it2!=bound_map.end()){
            if(it2!=bound_map.begin()) --traverse_it;
            while(1){
                if(temp_bound_x >= temp_x) break;
                if(traverse_it->second->Gp!=NULL && traverse_it->second->max_x <= temp_x &&  traverse_it->second->point_x > temp_bound_x ){
                    GP2->Add_edge(traverse_it->second->Gp, temp_x, temp_y2, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);
                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                if(traverse_it==bound_map.begin()) break;
                --traverse_it;
            }
        }
        else{
            //cout << "ritr";
            for(ritr = bound_map.rbegin();ritr!=bound_map.rend();++ritr){
                if(temp_bound_x >= temp_x) break;
                if(ritr->second->Gp!=NULL && ritr->second->max_x <= temp_x &&  ritr->second->point_x > temp_bound_x ){
                    GP2->Add_edge(ritr->second->Gp, temp_x, temp_y1, ritr->second->point_x, ritr->second->point_y, Layer_pos, Via_cost);
                }
                if(ritr->second->max_x > temp_bound_x) temp_bound_x = ritr->second->max_x;
            }  
        }

        //2
        temp_bound_x = min_x;
        traverse_it = it2;
        for(;traverse_it!=it1;++traverse_it){
            if(temp_bound_x >= temp_x) break;
            if(traverse_it->second->Gp!=NULL && traverse_it->second->max_x <= temp_x && traverse_it->second->point_x > temp_bound_x ){
                GP2->Add_edge(traverse_it->second->Gp, temp_x, temp_y2, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);
            }
            if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
        }

        //5 and 3 construct edge
        temp_bound_x = min_x;
        traverse_it = it1;
        for(;traverse_it!=bound_map.end();++traverse_it){
            if(temp_bound_x >= temp_x) break;
            if(traverse_it->second->Gp!=NULL && traverse_it->second->max_x <= temp_x && traverse_it->second->point_x > temp_bound_x ){
                GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);

            }
            if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
        }
        //3
        temp_bound_x = min_x;
        if(it1!=bound_map.end() ){
            traverse_it = it1;
            for(;traverse_it!=it2 && traverse_it!=bound_map.begin();--traverse_it){
                if(temp_bound_x >= temp_x) break;
                if(traverse_it->second->Gp!=NULL && traverse_it->second->max_x <= temp_x && traverse_it->second->point_x > temp_bound_x ){
                    GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);
                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
            }
        }
        else{
            //cout << "ritr";
            for(ritr = bound_map.rbegin();ritr!=bound_map.rend();++ritr){
                if(temp_bound_x >= temp_x || ritr->first <= temp_y2) break;
                if(ritr->second->Gp!=NULL && ritr->second->max_x <= temp_x && ritr->second->point_x > temp_bound_x ){
                    GP1->Add_edge(ritr->second->Gp, temp_x, temp_y1, ritr->second->point_x, ritr->second->point_y, Layer_pos, Via_cost);
                }
                if(ritr->second->max_x > temp_bound_x) temp_bound_x = ritr->second->max_x;
            }              
        }


    }


}

void
Layer::ConvertToUndirectedG(){
	pair<MAP_GP_edge::iterator, bool> map_gp_status;
	list < GraphPoint* >::iterator gp_itr,begin_itr,end_itr;
    list<Edge_info*>::iterator edge_itr;
    MAP_GP_edge::iterator map_gp_itr;
    GraphPoint *temp_gp;
    Edge_info *E1 = NULL;
    int x1,y1,x2,y2,distance;
    //status1 = bound_map.insert(pair< int , BoundLine_info*>(temp_y1, b1) );
 	for(size_t i = 0; i < all_cluster.size(); i++){
 		begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
            for(map_gp_itr = (*gp_itr)->map_edge.begin();map_gp_itr!=(*gp_itr)->map_edge.end(); ++map_gp_itr){
            	temp_gp = map_gp_itr->second->Gp;
                x1 = map_gp_itr->second->point_x1;
                y1 = map_gp_itr->second->point_y1;
                x2 = map_gp_itr->second->point_x2;
                y2 = map_gp_itr->second->point_y2;
                distance = map_gp_itr->second->distance;
            	map_gp_status = temp_gp->map_edge.insert(pair< int , Edge_info*>((*gp_itr)->idx, E1) );
            	if(map_gp_status.second==true){
					E1 = new Edge_info((*gp_itr), x2, y2, x1, y1, distance, map_gp_itr->second->layer);
					map_gp_status.first->second = E1;
				}
				else{
					if (distance < map_gp_status.first->second->distance){ //update point position & distance
						map_gp_status.first->second->point_x1 = x2;
						map_gp_status.first->second->point_y1 = y2;
						map_gp_status.first->second->point_x2 = x1;
						map_gp_status.first->second->point_y2 = y1;
						map_gp_status.first->second->distance = distance;
						map_gp_status.first->second->layer    = map_gp_itr->second->layer;
					}
				}

            }
        }
 	}

}

//int convert to string
string itos(int a) {
    string sign = a<0?"-":"";
    string result = a>0?string(1,(a%10+'0')):string(1,((a=-a)%10+'0'));
    (a/=10)>0?result=itos(a)+result:result;
    return sign+result;
 }



void 
Layer::check_point_svg(string name){
	cout << "START PLOT"<<endl;
    string a1 = string("Print")+name+string(".svg");
    ofstream a(a1.c_str());
    int size = 1;
    if(Width>10000) size = Width/10000;
    cout << "size:" << size <<endl;
    
    a << " <svg xmlns=\"http://www.w3.org/2000/svg\" width=\""<< Width/size <<"\" height=\""<< Height/size <<"\">\n ";
    a << "<marker xmlns=\"http://www.w3.org/2000/svg\" id=\"lineEnd\" viewBox=\"0 0 10 10\" refX=\"5\" refY=\"5\" markerUnits=\"strokeWidth\" markerWidth=\"4\" markerHeight=\"3\" orient=\"auto\">\n";
    a << "<rect x=\"0\" y=\"0\" width=\"10\" height=\"10\" fill=\"red\" />\n";
    a << "</marker>  " << endl;

    //shape
    //1.Routed shape
    list < Shape* >::iterator sh_itr,b_itr,e_itr;
    b_itr = Rshape_list.begin();
    e_itr = Rshape_list.end();
    int width,height;
    for(sh_itr = b_itr; sh_itr!=e_itr;++sh_itr){
        width = (*sh_itr)->coords->x2 - (*sh_itr)->coords->x1;
        height = (*sh_itr)->coords->y2 - (*sh_itr)->coords->y1;
        a << "<rect x=\"" << (*sh_itr)->coords->x1/size << "\" y=\"" << (*sh_itr)->coords->y1/size << "\" width=\"" << width/size << "\" height=\""<< height/size << "\" style=\"fill:blue;stroke:black;stroke-width:1;fill-opacity:0.2;stroke-opacity:0.6\" />" << endl;
    }
    //1.Obstacle shape
    b_itr = Obstacle_list.begin();
    e_itr = Obstacle_list.end();
    for(sh_itr = b_itr; sh_itr!=e_itr;++sh_itr){
        width = (*sh_itr)->coords->x2 - (*sh_itr)->coords->x1;
        height = (*sh_itr)->coords->y2 - (*sh_itr)->coords->y1;
        a << "<rect x=\"" << (*sh_itr)->coords->x1/size << "\" y=\"" << (*sh_itr)->coords->y1/size << "\" width=\"" << width/size << "\" height=\""<< height/size << "\" style=\"fill:gray;stroke:black;stroke-width:1;fill-opacity:0.2;stroke-opacity:0.6\" />" << endl;
    }

    //point
    list < GraphPoint* >::iterator gp_itr,begin_itr,end_itr;
    MAP_GP_edge::iterator map_gp_itr, map_begin_itr, map_end_itr;
    for(size_t i = 0; i < all_cluster.size(); i++){
        begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
            a<< "<circle cx=\"" << (*gp_itr)->x/size << "\" cy=\""<< (*gp_itr)->y/size << "\" r=\"2\" style=\"fill:red;stroke:red;stroke-width:3;fill-opacity:0.1;stroke-opacity:0.8\" />" << endl;
     
        }
    }
    //Via
    list < Shape* >::iterator via_itr;
    for(via_itr = Via_list.begin(); via_itr != Via_list.end(); ++via_itr){
        //if ((*via_itr)->clu->shape_list.size()>1)
        a<< "<circle cx=\"" << (*via_itr)->coords->x1/size << "\" cy=\""<< (*via_itr)->coords->y1/size << "\" r=\"5\" style=\"fill:red;stroke:red;stroke-width:4;fill-opacity:1;stroke-opacity:1\" />" << endl;
            
    }
    for(via_itr = Upper_Via_list.begin(); via_itr != Upper_Via_list.end(); ++via_itr){
        a<< "<circle cx=\"" << (*via_itr)->coords->x1/size << "\" cy=\""<< (*via_itr)->coords->y1/size << "\" r=\"5\" style=\"fill:black;stroke:black;stroke-width:4;fill-opacity:1;stroke-opacity:1\" />" << endl;
            
    }
    //edge
    int x1,y1,x2,y2;
    for(size_t i = 0; i < all_cluster.size(); i++){
        begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
        	int xx = (*gp_itr)->x;//test
        	int yy = (*gp_itr)->y;//test
            for(map_gp_itr = (*gp_itr)->map_edge.begin();map_gp_itr!=(*gp_itr)->map_edge.end(); ++map_gp_itr){
                x1 = map_gp_itr->second->point_x1;
                y1 = map_gp_itr->second->point_y1;
                x2 = map_gp_itr->second->point_x2;
                y2 = map_gp_itr->second->point_y2;
                if((*gp_itr)->Shape_type==OBSTACLE ){
                    if(xx!=x1 || yy!=y1) {
                        cout << "error! ";//something strange...";
                    }
                }
                if(map_gp_itr->second->layer==Layer_pos){
                    if(map_gp_itr->second->Gp->Layer_pos==Layer_pos+1)
                       ; //a << "<line x1=\"" << x1/size << "\" y1=\"" << y1/size << "\" x2=\"" << x2/size << "\" y2=\"" << y2/size << "\"\nstroke-width=\"2\" stroke=\"blue\"/>" << endl;
                    else if(map_gp_itr->second->Gp->Layer_pos==Layer_pos)
                       a << "<line x1=\"" << x1/size << "\" y1=\"" << y1/size << "\" x2=\"" << x2/size << "\" y2=\"" << y2/size << "\"\nstroke-width=\"2\" stroke=\"green\"/>" << endl;
                    else{
                       ;//a << "<line x1=\"" << x1/size << "\" y1=\"" << y1/size << "\" x2=\"" << x2/size << "\" y2=\"" << y2/size << "\"\nstroke-width=\"2\" stroke=\"purple\"/>" << endl;
                    }
                }
            }
        }
    }
    
    //check overlapping via
    /*int x1,y1,x2,y2;
    for(size_t i = 0; i < all_cluster.size(); i++){
        begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
            for(map_gp_itr = (*gp_itr)->map_edge.begin();map_gp_itr!=(*gp_itr)->map_edge.end(); ++map_gp_itr){
                x1 = map_gp_itr->second->point_x1;
                y1 = map_gp_itr->second->point_y1;
                x2 = map_gp_itr->second->point_x2;
                y2 = map_gp_itr->second->point_y2;
                if(x1!=x2 || y1!=y2) continue;

                if(map_gp_itr->second->layer==Layer_pos){
                    a<< "<circle cx=\"" << x2/size << "\" cy=\""<< y2/size << "\" r=\"5\" style=\"fill:black;stroke:black;stroke-width:4;fill-opacity:0.8;stroke-opacity:0.8\" />" << endl;
                }
            }
        }
    }*/
    
    //check Extended Dijkstra's
    /*int x1,y1,x2,y2;
    for(size_t i = 0; i < all_cluster.size(); i++){
 		begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
			int xx = (*gp_itr)->x;//test
        	int yy = (*gp_itr)->y;//test
            for(map_gp_itr = (*gp_itr)->map_edge.begin();map_gp_itr!=(*gp_itr)->map_edge.end(); ++map_gp_itr){
                x1 = map_gp_itr->second->point_x1;
                y1 = map_gp_itr->second->point_y1;
                x2 = map_gp_itr->second->point_x2;
                y2 = map_gp_itr->second->point_y2;
                if((*gp_itr)->Shape_type==OBSTACLE ){
                	if(xx!=x1 || yy!=y1) cin.get();//test error
                }
                if((*gp_itr)->parent==map_gp_itr->second->Gp ){
                    if(map_gp_itr->second->layer==Layer_pos){
                        if(map_gp_itr->second->Gp->Layer_pos==Layer_pos+1)
                           a << "<line x1=\"" << x1/size << "\" y1=\"" << y1/size << "\" x2=\"" << x2/size << "\" y2=\"" << y2/size << "\"\nstroke-width=\"3\" stroke=\"blue\"/>" << endl;
                        else if(map_gp_itr->second->Gp->Layer_pos==Layer_pos)
                           a << "<line x1=\"" << x1/size << "\" y1=\"" << y1/size << "\" x2=\"" << x2/size << "\" y2=\"" << y2/size << "\"\nstroke-width=\"3\" stroke=\"green\"/>" << endl;
                        else{
                           a << "<line x1=\"" << x1/size << "\" y1=\"" << y1/size << "\" x2=\"" << x2/size << "\" y2=\"" << y2/size << "\"\nstroke-width=\"3\" stroke=\"red\"/>" << endl;
                        }
                    }
                }

            }
        }
    }*/

    //check Extended Kruskal's
    /*int x1,y1,x2,y2;
    list<Edge_info*>::iterator edge_itr;
    for(size_t i = 0; i < all_cluster.size(); i++){
        begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
            int xx = (*gp_itr)->x;//test
            int yy = (*gp_itr)->y;//test
            for(edge_itr = (*gp_itr)->final_edge.begin();edge_itr!=(*gp_itr)->final_edge.end(); ++edge_itr){
                x1 = (*edge_itr)->point_x1;
                y1 = (*edge_itr)->point_y1;
                x2 = (*edge_itr)->point_x2;
                y2 = (*edge_itr)->point_y2;
                if((*gp_itr)->Shape_type==OBSTACLE ){
                    if(xx!=x1 || yy!=y1) cin.get();//test error
                }
                if((*edge_itr)->layer==Layer_pos){
                        if((*edge_itr)->Gp->Layer_pos==Layer_pos+1){
                            a << "<line x1=\"" << x1/size << "\" y1=\"" << y1/size << "\" x2=\"" << x2/size << "\" y2=\"" << y2/size << "\"\nstroke-width=\"3\" stroke=\"black\"/>" << endl;
                            //Via
                            a<< "<circle cx=\"" << x2/size << "\" cy=\""<< y2/size << "\" r=\"5\" style=\"fill:black;stroke:black;stroke-width:4;fill-opacity:0.8;stroke-opacity:0.8\" />" << endl;

                        }
                        else if((*edge_itr)->Gp->Layer_pos==Layer_pos){
                            a << "<line x1=\"" << x1/size << "\" y1=\"" << y1/size << "\" x2=\"" << x2/size << "\" y2=\"" << y2/size << "\"\nstroke-width=\"3\" stroke=\"green\"/>" << endl;
                        }
                        else{
                            a << "<line x1=\"" << x1/size << "\" y1=\"" << y1/size << "\" x2=\"" << x2/size << "\" y2=\"" << y2/size << "\"\nstroke-width=\"3\" stroke=\"red\"/>" << endl;
                            //Upper Via
                            a<< "<circle cx=\"" << x2/size << "\" cy=\""<< y2/size << "\" r=\"5\" style=\"fill:red;stroke:red;stroke-width:4;fill-opacity:0.8;stroke-opacity:0.8\" />" << endl;
                        }
                }
                else{
                    if((*edge_itr)->Gp->Layer_pos==Layer_pos+1){
                        a<< "<circle cx=\"" << x1/size << "\" cy=\""<< y1/size << "\" r=\"5\" style=\"fill:black;stroke:black;stroke-width:4;fill-opacity:0.8;stroke-opacity:0.8\" />" << endl;
                    }
                    else if((*edge_itr)->Gp->Layer_pos==Layer_pos-1){
                         a<< "<circle cx=\"" << x1/size << "\" cy=\""<< y1/size << "\" r=\"5\" style=\"fill:red;stroke:red;stroke-width:4;fill-opacity:0.8;stroke-opacity:0.8\" />" << endl;

                    }
                }

            }
        }
    }*/
    //debug



    a << "</svg>" << endl;
    a.close();

// <line x1="20" y1="100" x2="100" y2="20"
//      stroke-width="1" stroke="black"/>
}


























void Layer::Rshape_list_append(Shape *temp_shape){
	Rshape_num++;
	Rshape_list.push_back(temp_shape);
	//X_msort_shape.insert(MAP_Shape::value_type(temp_shape->coords->x2, temp_shape));
	//Y_msort_shape.insert(MAP_Shape::value_type(temp_shape->coords->y2, temp_shape));
}

void Layer::Obstacle_list_append(Shape *temp_shape){
	Obstacle_num++;
	Obstacle_list.push_back(temp_shape);
	//X_msort_shape.insert(MAP_Shape::value_type(temp_shape->coords->x2, temp_shape));
	//Y_msort_shape.insert(MAP_Shape::value_type(temp_shape->coords->y2, temp_shape));
}

void Layer::Via_list_append(Shape *temp_via){
	Via_num++;
	Via_list.push_back(temp_via);
}

void Layer::Upper_Via_list_append(Shape *temp_via){
    Upper_Via_num++;
    Upper_Via_list.push_back(temp_via);
}

int Layer::get_Rshape_num(){
	return Rshape_num;
}

int Layer::get_Obstacle_num(){
	return Obstacle_num;
}

int Layer::get_Via_num(){
	return Via_num;
}
