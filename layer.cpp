#include "layer.h"
#include <iostream>
#include <algorithm>
#include <fstream>

using namespace std;


Layer::Layer()
:Rshape_num(0),Obstacle_num(0),Via_num(0),Layer_Shape_num(0),G_point_num(0) //initialize
{
	//cout << "QQ";
}

bool Sort_Shape(Shape* a, Shape* b){
	return a->coords->x1 < b->coords->x1;
}



void Layer::SpanningGraphConstruct(){
	/* 1.1 construct cluster

	   */
	//#1.0 construct all_shape_vec
	int pos = 0;
	Layer_Shape_num = Rshape_num+Obstacle_num;
	all_shape_vec.resize(Layer_Shape_num+Via_num);
	for (it_shape it_s= Rshape_list.begin();it_s != Rshape_list.end();++it_s)
		all_shape_vec[pos++] = (*it_s); 
	for (it_shape it_s= Obstacle_list.begin();it_s != Obstacle_list.end();++it_s)
		all_shape_vec[pos++] = (*it_s); 
    for (it_shape it_s= Via_list.begin();it_s != Via_list.end();++it_s)
        all_shape_vec[pos++] = (*it_s);    
	
	//#1 construct cluster
    clustering_shape();

    //#2 construct graph 
    //SGconstruct();

    //check_cluster();
	//for(MS_it it = X_msort_shape.begin();it!=X_msort_shape.end();++it)
	

}

void Layer::SpanningTreeConstruct(){

	//#1 Extended Dijkstra's Algorithm
    ExtendedDijkstra();

    //#2 Extended Kruskal's Algorithm
    ExtendedKruskal();

    //#PLOT
    check_point_svg();

}



bool
sort_func_x1(Shape* S1, Shape* S2){
	if(S1->coords->x1 < S2->coords->x1) return true;
	else if(S1->coords->x1 == S2->coords->x1) return (S1->coords->y1 < S2->coords->y1);
	else return false;
}

void
Layer::clustering_shape(){
	cout<<"Clustering..."<<endl;
	for(size_t s = 0; s < all_shape_vec.size(); s++){
		all_shape_vec[s]->clu = NULL;
		sort_by_x1.push_back(all_shape_vec[s]);
	}
    sort(sort_by_x1.begin(), sort_by_x1.end(), sort_func_x1);
    cout<<"layer:"<<endl;
    /*cout<<"sort by x1:"<<endl;
    for(size_t s = 0; s < sort_by_x1.size(); s++)
    	cout<<s<<": "<<sort_by_x1[s]->coords->x1<<", "<<sort_by_x1[s]->coords->y1<<endl;*/
    
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
    	//cout<<"C"<<i<<": ";
    	Shape* S = all_cluster[i]->shape_list.front();
    	//cout<<S->coords->x1<<", "<<S->coords->y1<<"  "<<all_cluster[i]->shape_list.size()<<endl;
    	if(S->Shape_type == RSHAPE)count_S ++;
    	else count_O ++;
    	/*it_shape it = all_cluster[i]->shape_list.begin();
    	cout<<"elements:"<<endl;
    	for(; it != all_cluster[i]->shape_list.end(); it++)
    		cout<<(*it)->coords->x1<<", "<<(*it)->coords->y1<<"; "
    	    <<(*it)->coords->x2<<", "<<(*it)->coords->y2<<endl;*/
    }
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
    GraphPoint *GP1, *GP2;
    Cluster *temp_clu;

    /*
                    |
              5     |
        ---------------------------- p1
              3     |               |
     del 4          |  L (shape)    |              
              2     |               |
        ---------------------------- p2
              1     |
                    |

    */

    temp_x = LLine->x;
    temp_max_x = LLine->S->coords->x2;
    temp_clu = LLine->S->clu;
    temp_y1 = LLine->y+LLine->length;
    temp_y2 = LLine->y;

    if(LLine->S->Shape_type==VIA){
        cerr << "via~";
        p1 = true;
        GP1 = NULL;
        BoundLine_info* b1 = new BoundLine_info(LLine->LR, temp_max_x, temp_x, UP, temp_y1);
        b1->up_edge_x = b1->down_edge_x = -1;

        status1 = bound_map.insert(pair< int , BoundLine_info*>(temp_y1, b1) );
        it1 = status1.first;
        
        //############check point whether need to insert
        traverse_it = it1;
        ++traverse_it;

        if(status1.second==false && it1->second->max_x >= temp_x ){ //same y 
            //up point no need 
        }
        else if(status1.second!=false && traverse_it!=bound_map.end() && traverse_it->second->down_edge_x >= temp_x){
            //up point no need 
            if( traverse_it->second->down_edge_x >= temp_max_x) p1 = false;//this point need to delete
        }
        else{
            GP1 = temp_clu->Add_GP(LLine, UP, G_point_num);
            //1 and 5 construct edge
            temp_bound_x = -1;
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
           temp_bound_x = -1;
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
            if(it1->second->max_x <= temp_x){
                it1->second->point_x = temp_x;
                it1->second->point_y = temp_y1;
            }
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
                it1->second->down_edge_x = traverse_it->second->up_edge_x;
            }
        }

    }
    else if(LLine->LR==LEFT){ //LEFT
        if(temp_y1==temp_y2){
            cerr << temp_x<<" " << temp_y1;
            cin.get();
        }
        p1 = p2 = true;
        GP1 = GP2 = NULL;
        BoundLine_info* b1 = new BoundLine_info(LLine->LR, temp_max_x, temp_x, UP, temp_y1);
        BoundLine_info* b2 = new BoundLine_info(LLine->LR, temp_max_x, temp_x, DOWN, temp_y2);

        status1 = bound_map.insert(pair< int , BoundLine_info*>(temp_y1, b1) );
        it1 = status1.first;
        status2 = bound_map.insert(pair< int , BoundLine_info*>(temp_y2, b2) );
        it2 = status2.first;
        
        //############check it2 whether need to insert
        traverse_it = it2;
        if(traverse_it!=bound_map.begin()) --traverse_it;

        if(status2.second==false && it2->second->max_x >= temp_x ){ //same y 
            //Down point no need 
        }
        else if(status2.second!=false && it2!=bound_map.begin() && traverse_it->second->up_edge_x >= temp_x){
            //Down point no need 
            if(traverse_it->second->up_edge_x >= temp_max_x) p2 = false;//dont inert this x line 
        }
        else{
            GP2 = temp_clu->Add_GP(LLine, DOWN, G_point_num);
            //1 and 2 construct edge
            temp_bound_x = -1;
            traverse_it = it2;
            if(status2.second==false){
                if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                    GP2->Add_edge(traverse_it->second->Gp, temp_x, temp_y2, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, 0);
                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
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
            temp_bound_x = -1;
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

        if(status1.second==false && it1->second->max_x >= temp_x ){ //same y 
            //up point no need 
        }
        else if(status1.second!=false && traverse_it!=bound_map.end() && traverse_it->second->down_edge_x >= temp_x){
            //up point no need 
            if( traverse_it->second->down_edge_x >= temp_max_x) p1 = false;//this point need to delete
        }
        else{
            GP1 = temp_clu->Add_GP(LLine, UP, G_point_num);
            
            //3 and 5 construct edge
            temp_bound_x = -1;
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
            temp_bound_x = -1;
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
                if(traverse_it->second->max_x >= temp_x ) connect = false;
            }
            if(connect) GP1->Add_edge(GP2, temp_x, temp_y1, temp_x, temp_y2, Layer_pos, 0);
        }
        
        
        //############delete mid
        traverse_it = it2;
        ++traverse_it;
        if(it2->first!=it1->first)//if the shape is not a wire
            for(;traverse_it!=it1; ++traverse_it){

                if(traverse_it->second->max_x <= temp_max_x ) {
                    bound_map.erase(traverse_it++);//traverse_it = bound_map.erase(traverse_it);
                    --traverse_it;

                }
                else{
                    if(traverse_it->second->up_edge_x < temp_max_x)   traverse_it->second->up_edge_x   = temp_max_x;
                    if(traverse_it->second->down_edge_x < temp_max_x) traverse_it->second->down_edge_x = temp_max_x;
                    traverse_it->second->Gp = NULL;
                }
            }
        if(it1->second->down_edge_x < temp_max_x) it1->second->down_edge_x = temp_max_x;// && status1.second==false 
        if(it2->second->up_edge_x < temp_max_x) it2->second->up_edge_x = temp_max_x;// && status2.second==false
        
        //############ bug?
        if(status1.second==false){
            if(it1->second->max_x <= temp_x){
                it1->second->point_x = temp_x;
                it1->second->point_y = temp_y1;
            }
            if(it1->second->max_x<temp_max_x)
                it1->second->max_x = temp_max_x;
        }
        if(status2.second==false) {
            if(it2->second->max_x <= temp_x){
                it2->second->point_x = temp_x;
                it2->second->point_y = temp_y2;
            }
            if(it2->second->max_x<temp_max_x)
                it2->second->max_x = temp_max_x;
        }

        //
        if(p1==false) bound_map.erase(it1);
        else{
            traverse_it = it1;
            ++traverse_it;
            if(traverse_it!=bound_map.end())
                it1->second->up_edge_x = traverse_it->second->down_edge_x;
        }
        if(p2==false) bound_map.erase(it2);
        else{
            if(it2!=bound_map.begin()) {
                traverse_it = it2;
                --traverse_it;
                it2->second->down_edge_x = traverse_it->second->up_edge_x;
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
	
	if(GP1==NULL && GP2==NULL) return;

    map< int , BoundLine_info* , less<int> >::iterator it1,it2, low_it,traverse_it;
    pair<map< int , BoundLine_info* , less<int> >::iterator,bool> status1,status2;
    GraphPoint *_GP;
    bool p1,p2; //t_shape_type,
    int temp_x,temp_max_x, temp_bound_x,temp_y1,temp_y2;
    Cluster *temp_clu;

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
    temp_max_x = LLine->S->coords->x2;
    temp_clu = LLine->S->clu;
    temp_y1 = LLine->y+LLine->length;
    temp_y2 = LLine->y;

    if(LLine->S->Shape_type==VIA || GP1==NULL || GP2==NULL){
    	if(GP1==NULL) {
    		_GP = GP2;
    		temp_y1 = temp_y2;
    	}
    	else {
    		 _GP = GP1;
    	}

        it1 = bound_map.lower_bound(temp_y1);
        cout << "yo";
        //1 and 5 construct edge
        temp_bound_x = -1;
        traverse_it = it1;
        for(;traverse_it!=bound_map.end();++traverse_it){
            if(temp_bound_x >= temp_x) break;
            if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                _GP->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);

            }
            if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
        }

        //1
        temp_bound_x = -1;
        traverse_it = it1;
        while(1){
            if(temp_bound_x >= temp_x) break;
            if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                _GP->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);
            }
            if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
            if(traverse_it==bound_map.begin()) break;
            --traverse_it;
        }

        
    }
    return;
    /*else if(LLine->LR==LEFT){ //LEFT
        p1 = p2 = true;

        it2 = bound_map.lower_bound(temp_y2);//
        
        //1 and 2 construct edge
        temp_bound_x = -1;
        traverse_it = it2;
        if(status2.second==false){
            if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                GP2->Add_edge(traverse_it->second->Gp, temp_x, temp_y2, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);
            }
            if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
        }
        if(it2!=bound_map.begin()) --traverse_it;
        while(1){
            if(temp_bound_x >= temp_x) break;
            if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                GP2->Add_edge(traverse_it->second->Gp, temp_x, temp_y2, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);
            }
            if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
            if(traverse_it==bound_map.begin()) break;
            --traverse_it;
        }
        //2
        temp_bound_x = -1;
        traverse_it = it2;
        ++traverse_it;
        for(;traverse_it!=it1;++traverse_it){
            if(temp_bound_x >= temp_x) break;
            if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                GP2->Add_edge(traverse_it->second->Gp, temp_x, temp_y2, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);
            }
            if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
        }

        it2->second->Gp = GP2;

        

        //############check it1  whether need to insert
        traverse_it = it1;
        ++traverse_it;

        if(status1.second==false && it1->second->max_x >= temp_x ){ //same y 
            //up point no need 
        }
        else if(status1.second!=false && traverse_it!=bound_map.end() && traverse_it->second->down_edge_x >= temp_x){
            //up point no need 
            if( traverse_it->second->down_edge_x >= temp_max_x) p1 = false;//this point need to delete
        }
        else{
            GP1 = temp_clu->Add_GP(LLine, UP, G_point_num);
            
            //3 and 5 construct edge
            temp_bound_x = -1;
            traverse_it = it1;
            ++traverse_it;
            for(;traverse_it!=bound_map.end();++traverse_it){
                if(temp_bound_x >= temp_x) break;
                if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                    GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);

                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
            }
            //3
            temp_bound_x = -1;
            traverse_it = it1;
            if(status1.second==false){
                if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                    GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);
                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
            }
            --traverse_it;
            for(;traverse_it!=it2;--traverse_it){
                if(temp_bound_x >= temp_x) break;
                if(traverse_it->second->Gp!=NULL && traverse_it->second->point_x > temp_bound_x ){
                    GP1->Add_edge(traverse_it->second->Gp, temp_x, temp_y1, traverse_it->second->point_x, traverse_it->second->point_y, Layer_pos, Via_cost);
                }
                if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
            }

            it1->second->Gp = GP1;//////

        }

    }*/


}




void
Layer::ConvertToUndirectedG(){
	pair<MAP_GP_edge::iterator, bool> map_gp_status;
	list < GraphPoint* >::iterator gp_itr,begin_itr,end_itr;
    MAP_GP_edge::iterator map_gp_itr;
    GraphPoint *temp_gp;
    Edge_info *E1;
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
					E1 = new Edge_info((*gp_itr), x2, y2, x1, y1, distance, Layer_pos);
					map_gp_status.first->second = E1;
				}
				else{
					if (distance < map_gp_status.first->second->distance){ //update point position & distance
						map_gp_status.first->second->point_x1 = x2;
						map_gp_status.first->second->point_y1 = y2;
						map_gp_status.first->second->point_x2 = x1;
						map_gp_status.first->second->point_y2 = y1;
						map_gp_status.first->second->distance = distance;
						map_gp_status.first->second->layer    = Layer_pos;
					}
				}

            }
        }
 	}

}


void 
Layer::SG_find_GPinfo(int x_pos, BoundLine_info* bl_info){
	

}

void
Layer::ExtendedDijkstra(){

	FibQueue<int> Fq;
	FibHeap<int>::FibNode *temp_fibn;
	list < GraphPoint* >::iterator gp_itr,begin_itr,end_itr;
    MAP_GP_edge::iterator map_gp_itr;
    GraphPoint *temp_gp, *temp_gp2;
    int temp_dis, temp_dis2;


	//### 1. insert all GP in fibo heap & initialize SET
	for(size_t i = 0; i < all_cluster.size(); i++){
 		begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        if(all_cluster[i]->GetShapeType()==RSHAPE){
        	(*begin_itr)->parent = (*begin_itr); //SET root
        	(*begin_itr)->terminal_dis = 0;
        	(*begin_itr)->Fnode = Fq.push(0, (*begin_itr));
        	(*begin_itr)->select = false;
        }
        else{
        	for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
        		(*gp_itr)->parent = NULL;
        		(*gp_itr)->terminal_dis = INT_MAX;
        		(*gp_itr)->Fnode = Fq.push(INT_MAX, (*gp_itr));
        		(*gp_itr)->select = false;
        	}
        }
    }

    //### 2. Shortest path terminal forest construct
    while(!Fq.empty()){
		temp_fibn = Fq.topNode();
		temp_dis = temp_fibn->key;
		temp_gp = (GraphPoint*)temp_fibn->payload;
		temp_gp->select = true;
		//cout << "Top: " << temp_dis << endl;

		//# pop the target vertex
		Fq.pop();

		//# update edge   // map_gp_itr->second->Gp .... not good
		for(map_gp_itr = temp_gp->map_edge.begin();map_gp_itr!=temp_gp->map_edge.end(); ++map_gp_itr){
			temp_gp2 = map_gp_itr->second->Gp;
			temp_dis2 = map_gp_itr->second->distance + temp_dis;
			if( temp_gp2->select==false && temp_gp2->terminal_dis > temp_dis2 ){
				temp_gp2->terminal_dis = temp_dis2;
				temp_gp2->parent = temp_gp;

				Fq.decrease_key(temp_gp2->Fnode, temp_dis2);
			}

		}

    }

    //### 3. Find Set
    int num_vertex=0;
	for(size_t i = 0; i < all_cluster.size(); i++){
 		begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
        	 (*gp_itr)->root = (*gp_itr)->Find_Set();
        	 num_vertex++;
        	 //bug
        	 if((*gp_itr)->root==NULL){
                string sshape;
                if((*gp_itr)->Shape_type==RSHAPE) sshape = "RSHAPE";
                else                              sshape = "OBSTACLE";
                cerr << "A " << sshape << "'s vertex is isolated\n";
        	 	for(map_gp_itr = (*gp_itr)->map_edge.begin();map_gp_itr!=(*gp_itr)->map_edge.end(); ++map_gp_itr){
        	 		cout << "a"<< (*gp_itr)->idx;
        	 		cout <<"\n" <<  map_gp_itr->second->distance << endl;
        	 	}
        	 	cout << endl;
        	 }
        }
    }

    //#check
    //cout << "num vertex: " << num_vertex << endl;


}

GraphPoint* findSet(GraphPoint *p) {
	if (p != p->parentKK)
		p->parentKK = findSet(p->parentKK);
	return p->parentKK;
}

void unionSet( GraphPoint *s1, GraphPoint *s2 ) {

	if (s1->rank >= s2->rank)
		s2->parentKK = s1;
	else 
		s1->parentKK = s2;

	if (s1->rank == s2->rank)
		s1->rank++;

}

void markChosenPoints(GraphPoint *p1, GraphPoint *p2) {
	p1->chosen = p2->chosen = true;
	GraphPoint *p = p1;
	while(p != p->parent) {
		p->chosen = true;
		p = p->parent;
	}
	p->chosen = true;
	p = p2;
	while(p != p->parent) {
		p->chosen = true;
		p = p->parent;
	}
	p->chosen = true;
}

void Layer::addMSTEdges(GraphPoint *p1, GraphPoint *p2) {
	GraphPoint *p = p1;
    int x1,y1,x2,y2;
    MAP_GP_edge::iterator map_gp_itr, map_begin_itr, map_end_itr;
    for(map_gp_itr = p1->map_edge.begin();map_gp_itr!=p1->map_edge.end(); ++map_gp_itr){
        x1 = map_gp_itr->second->point_x1;
        y1 = map_gp_itr->second->point_y1;
        x2 = map_gp_itr->second->point_x2;
        y2 = map_gp_itr->second->point_y2;
        if(p2==map_gp_itr->second->Gp ) MSTEdges.push_back( Edge(x1, x2, y1, y2) );
    }

	while (p != p->parent) {
		for(map_gp_itr = p->map_edge.begin();map_gp_itr!=p->map_edge.end(); ++map_gp_itr){
            x1 = map_gp_itr->second->point_x1;
            y1 = map_gp_itr->second->point_y1;
            x2 = map_gp_itr->second->point_x2;
            y2 = map_gp_itr->second->point_y2;
            if(p->parent==map_gp_itr->second->Gp ) MSTEdges.push_back( Edge(x1, x2, y1, y2) );
        }
		p = p->parent;
	}
	p = p2;
	while (p != p->parent) {
	    for(map_gp_itr = p->map_edge.begin();map_gp_itr!=p->map_edge.end(); ++map_gp_itr){
            x1 = map_gp_itr->second->point_x1;
            y1 = map_gp_itr->second->point_y1;
            x2 = map_gp_itr->second->point_x2;
            y2 = map_gp_itr->second->point_y2;
            if(p->parent==map_gp_itr->second->Gp ) MSTEdges.push_back( Edge(x1, x2, y1, y2) );
        }
		p = p->parent;
	}
}

void Layer::ExtendedKruskal() {
	cout << "...Start Kruskal's" << endl;
	list <GraphPoint*>::iterator gp_itr, begin1, end1;
	MAP_GP_edge::iterator map_gp_itr, begin2, end2;
	GraphPoint *temp_gp1, *temp_gp2;
	int edgeLength, dis1, dis2;

	multimap < int, Edge_info* > HeapBE; // bridge edge
	multimap < int, Edge_info* >::iterator curEdge;

	// initialization
    int ttt= 0;
    for (size_t i = 0; i < all_cluster.size(); i++) {
 		begin1 = all_cluster[i]->GraphP_list.begin();
        end1 = all_cluster[i]->GraphP_list.end();
        if(all_cluster[i]->GetShapeType()==RSHAPE){
             (*(all_cluster[i]->GraphP_list.begin() ))->parentKK = (*(all_cluster[i]->GraphP_list.begin() ));
             ttt++;
        }
        for (gp_itr = begin1; gp_itr != end1; ++gp_itr) {
        	temp_gp1 = (*gp_itr);
        	temp_gp1->parentKK = temp_gp1;
        	//temp_gp1->visit = true;
        	begin2 = temp_gp1->map_edge.begin();
        	end2 = temp_gp1->map_edge.end();

            for (map_gp_itr = begin2; map_gp_itr != end2; ++map_gp_itr) {
            	temp_gp2 = map_gp_itr->second->Gp;
            	//if (temp_gp2->visit == true) continue;
            	edgeLength = map_gp_itr->second->distance;
            	dis1 = temp_gp1->terminal_dis;
            	dis2 = temp_gp2->terminal_dis;

            	if (temp_gp1->root != temp_gp2->root) {
            		map_gp_itr->second->source = temp_gp1;
            		HeapBE.insert(  pair <int, Edge_info*> ( dis1 + edgeLength + dis2, map_gp_itr->second )  );
            	}
            }
        }
    }

    cout << "...Start choosing points" << endl;
 	// choose MST points
    while (!HeapBE.empty()) {
    	curEdge = HeapBE.begin();
    	temp_gp1 = curEdge->second->source;
    	temp_gp2 = curEdge->second->Gp;
    	GraphPoint *set1 = findSet(temp_gp1->root);
    	GraphPoint *set2 = findSet(temp_gp2->root);
        //cout << "shape11 type:" << temp_gp1->Shape_type << ", " << temp_gp2->Shape_type << endl;
        //cout << "root: "<< temp_gp1->root << ", " << temp_gp2->root << endl;
    	if (set1 != set2) {
    		unionSet(set1, set2);
    		addMSTEdges(temp_gp1, temp_gp2);
    	}
    	HeapBE.erase(curEdge);
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
Layer::check_point_svg(){
	cout << "START PLOT"<<endl;
    string a1 = string("Point_test")+string(".svg");
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
        //cout << "via"<<endl;
        if ((*via_itr)->clu->shape_list.size()>1)
        a<< "<circle cx=\"" << (*via_itr)->coords->x1/size << "\" cy=\""<< (*via_itr)->coords->y1/size << "\" r=\"5\" style=\"fill:red;stroke:red;stroke-width:4;fill-opacity:1;stroke-opacity:1\" />" << endl;
        else
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
                	if(xx!=x1 || yy!=y1) cin.get();//test error
                }
                //cout << map_gp_itr->second->layer;
                a << "<line x1=\"" << x1/size << "\" y1=\"" << y1/size << "\" x2=\"" << x2/size << "\" y2=\"" << y2/size << "\"\nstroke-width=\"2\" stroke=\"green\"/>" << endl;

            }
        }
    }
    
    /*
    //check Extended Dijkstra's
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
                	if(xx!=x1 || yy!=y1) cin.get();//test error
                }
                if((*gp_itr)->parent==map_gp_itr->second->Gp )
                	a << "<line x1=\"" << x1/size << "\" y1=\"" << y1/size << "\" x2=\"" << x2/size << "\" y2=\"" << y2/size << "\"\nstroke-width=\"2\" stroke=\"green\"/>" << endl;
            }
        }
    }*/

    //check Extended Kruskal's
    /*int x1,y1,x2,y2;
    for (auto itr = MSTEdges.begin(); itr != MSTEdges.end(); ++itr) {
    	x1 = (*itr)._x1;
    	x2 = (*itr)._x2;
    	y1 = (*itr)._y1;
    	y2 = (*itr)._y2;
    	a << "<line x1=\"" << x1/size << "\" y1=\"" << y1/size << "\" x2=\"" << x2/size << "\" y2=\"" << y2/size << "\"\nstroke-width=\"2\" stroke=\"green\"/>" << endl;
    }*/


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

int Layer::get_Rshape_num(){
	return Rshape_num;
}

int Layer::get_Obstacle_num(){
	return Obstacle_num;
}

int Layer::get_Via_num(){
	return Via_num;
}

/*
void //old version
Layer::SGconstruct(){

        //#######check
        for (it1 = bound_map.begin();it1 != bound_map.end();++it1){
            //cout << "y: " << it1->first <<" , max_x: " << it1->second->max_x << ", upper:" <<  it1->second->up_edge_x << ",down:" <<it1->second->down_edge_x << endl;
        }
        
        it1 = bound_map.begin();
        int pre = it1->second->up_edge_x;
        int temp_x;
        ++it1;
        for (;it1 != bound_map.end();++it1){
            temp_x = it1->second->down_edge_x;
            if(temp_x!=pre){
                cerr << "fuck u " << s << endl;
                cerr << it1->first;
                cin.get();
            }
            pre = it1->second->up_edge_x;
        }

        //#####

    //### 3. directed graph convert to undirected grpah
    ConvertToUndirectedG();

    //### 4. Print 
    cout << "point:" <<G_point_num << endl;
    cout << "shape count:" << Layer_Shape_num << endl;

}
*/


