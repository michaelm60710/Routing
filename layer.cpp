#include "layer.h"
#include <iostream>
#include <algorithm>
#include <fstream>

using namespace std;


Layer::Layer()
:Rshape_num(0),Obstacle_num(0),Via_num(0),Layer_Shape_num(0) //initialize
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
	all_shape_vec.resize(Layer_Shape_num);
	for (it_shape it_s= Rshape_list.begin();it_s != Rshape_list.end();++it_s)
		all_shape_vec[pos++] = (*it_s); 
	for (it_shape it_s= Obstacle_list.begin();it_s != Obstacle_list.end();++it_s)
		all_shape_vec[pos++] = (*it_s); 
    
	
	//#1.1 construct cluster
    clustering_shape();

    //#2 construct graph
    SGconstruct();

    //check_cluster();
	//for(MS_it it = X_msort_shape.begin();it!=X_msort_shape.end();++it)
	

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

void
Layer::SGconstruct(){
    cout << "SGconstruct"<< endl;
    vector <Line* > all_line_vec;
    all_line_vec.resize(Layer_Shape_num*2);

    //### 1. sorting all shape line(x)
    for(size_t s = 0; s < all_shape_vec.size(); s++){
        Line *l_l = new Line;
        Line *l_r = new Line;
        l_l->S = l_r->S = all_shape_vec[s];
        l_l->x = all_shape_vec[s]->coords->x1;
        l_r->x = all_shape_vec[s]->coords->x2;
        l_l->y = l_r->y = all_shape_vec[s]->coords->y1;
        l_l->length = l_r->length = all_shape_vec[s]->coords->y2 - l_l->y;
        l_l->LR = LEFT;
        l_r->LR = RIGHT;
        //cout << "x,y,x2,y2:"<< all_shape_vec[s]->coords->x1 << " " << all_shape_vec[s]->coords->y1 << " " << all_shape_vec[s]->coords->x2 << " " <<all_shape_vec[s]->coords->y2 << endl;
        //cout << "l_l:" << l_l->x << " " << l_l->y << " " << l_l->length << " " <<  l_l->LR << endl;
        //cout << "l_r:" << l_r->x << " " << l_r->y << " " << l_r->length << " " <<  l_r->LR << endl;
        all_line_vec[2*s] = l_l;
        all_line_vec[2*s+1] = l_r;
    }
    sort(all_line_vec.begin(), all_line_vec.end(), sort_line_x);

    //### 2. lets go~
    /*
                |
          5     |
    ------------------------
          3     | 
 del 4          |  L (shape)
          2     |
    ------------------------
          1     |
                |

    */


    map< int , BoundLine_info* , less<int> > bound_map;
    map< int , BoundLine_info* , less<int> >::iterator it1,it2, low_it,traverse_it;
    pair<map< int , BoundLine_info* , less<int> >::iterator,bool> status1,status2;
    bool t_shape_type,p1,p2;
    int temp_x,temp_max_x, temp_bound_x;
    GraphPoint *GP1, *GP2;
    Cluster *temp_clu;
    G_point_num = 0;

    for(size_t s = 0; s < all_line_vec.size(); s++){
        //if(s>224) break;

    	t_shape_type = all_line_vec[s]->S->Shape_type;
        temp_x = all_line_vec[s]->x;
        temp_max_x = all_line_vec[s]->S->coords->x2;
        temp_clu = all_line_vec[s]->S->clu;
        //cerr << s << ": temp_max_x: " <<  temp_max_x << ", bound size:" << bound_map.size() << endl;
        //cerr << "x:" << all_line_vec[s]->x << ",max_x:" << temp_max_x << ",y: " <<all_line_vec[s]->y <<"LR:" << all_line_vec[s]->LR<< endl;

    	if(all_line_vec[s]->LR==LEFT){ //LEFT
            p1 = p2 = true;
    		BoundLine_info* b1 = new BoundLine_info(all_line_vec[s]->LR, temp_max_x, UP);
    		BoundLine_info* b2 = new BoundLine_info(all_line_vec[s]->LR, temp_max_x, DOWN);

    		status1 = bound_map.insert(pair< int , BoundLine_info*>(all_line_vec[s]->y+all_line_vec[s]->length, b1) );
            it1 = status1.first;
    		status2 = bound_map.insert(pair< int , BoundLine_info*>(all_line_vec[s]->y                        , b2) );
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
                GP2 = new GraphPoint(all_line_vec[s], DOWN, G_point_num++);
                temp_clu->Add_GP(GP2);
                //1 and 2 construct edge
                temp_bound_x = -1;
                traverse_it = it2;
                if(status2.second==false){
                    if(traverse_it->second->Gp!=NULL && traverse_it->second->Gp->x > temp_bound_x ){
                        GP2->Add_edge(traverse_it->second->Gp);   
                    }
                    if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                }
                if(it2!=bound_map.begin()) --traverse_it;
                for(;traverse_it!=bound_map.begin();--traverse_it){
                    if(temp_bound_x >= temp_x) break;
                    if(traverse_it->second->Gp!=NULL && traverse_it->second->Gp->x > temp_bound_x ){
                        GP2->Add_edge(traverse_it->second->Gp);
                        
                    }
                    if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                }
                //2
                temp_bound_x = -1;
                traverse_it = it2;
                ++traverse_it;
                for(;traverse_it!=it1;++traverse_it){
                    if(temp_bound_x >= temp_x) break;
                    if(traverse_it->second->Gp!=NULL && traverse_it->second->Gp->x > temp_bound_x ){
                        GP2->Add_edge(traverse_it->second->Gp);
                        
                    }
                    if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                }

                it2->second->Gp = GP2;
            }

            //############check it1  whether need to insert
            traverse_it = it1;
            ++traverse_it;

            if(status1.second==false && it1->second->max_x >= temp_x ){ //same y 
                //Down point no need 
            }
            else if(status1.second!=false && traverse_it!=bound_map.end() && traverse_it->second->down_edge_x >= temp_x){
                //Down point no need 
                if( traverse_it->second->down_edge_x >= temp_max_x) p1 = false;
            }
            else{
                GP1 = new GraphPoint(all_line_vec[s], UP, G_point_num++);
                temp_clu->Add_GP(GP1);
                
                //3 and 5 construct edge
                temp_bound_x = -1;
                traverse_it = it1;
                ++traverse_it;
                for(;traverse_it!=bound_map.end();++traverse_it){
                    if(temp_bound_x >= temp_x) break;
                    if(traverse_it->second->Gp!=NULL && traverse_it->second->Gp->x > temp_bound_x ){
                        GP1->Add_edge(traverse_it->second->Gp);
                        
                    }
                    if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                }
                //3
                temp_bound_x = -1;
                traverse_it = it1;
                if(status1.second==false){
                    if(traverse_it->second->Gp!=NULL && traverse_it->second->Gp->x > temp_bound_x ){
                        GP1->Add_edge(traverse_it->second->Gp);
                        
                    }
                    if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                }
                --traverse_it;
                for(;traverse_it!=it2;--traverse_it){
                    if(temp_bound_x >= temp_x) break;
                    if(traverse_it->second->Gp!=NULL && traverse_it->second->Gp->x > temp_bound_x ){
                        GP1->Add_edge(traverse_it->second->Gp);
                        
                    }
                    if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                }
    
                it1->second->Gp = GP1;//////

            }

            //############delete mid
            traverse_it = it2;
            ++traverse_it;
            for(;traverse_it!=it1; ++traverse_it){

                if(traverse_it->second->max_x <= temp_max_x ) {
                    traverse_it = bound_map.erase(traverse_it);
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


            if(status1.second==false && it1->second->max_x<temp_max_x) it1->second = b1;
            if(status2.second==false && it2->second->max_x<temp_max_x) it2->second = b2;
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

            it1 = bound_map.lower_bound(all_line_vec[s]->y+all_line_vec[s]->length);
            it2 = bound_map.lower_bound(all_line_vec[s]->y);
            //############check it2 whether exist
            for (traverse_it = it2; traverse_it!=bound_map.end();++traverse_it){
                if(traverse_it->first > all_line_vec[s]->y) break;
                if(traverse_it->first == all_line_vec[s]->y && traverse_it->second->max_x == temp_x ){
                    p2 = true;
                    break;
                }
            }            
            if(p2){
                GP2 = new GraphPoint(all_line_vec[s], DOWN, G_point_num++);
                temp_clu->Add_GP(GP2);
                //1 construct edge
                temp_bound_x =  all_line_vec[s]->S->coords->x1;
                traverse_it = it2;
                if(it2!=bound_map.begin()) --traverse_it;
                for(;traverse_it!=bound_map.begin();--traverse_it){
                    if(temp_bound_x > temp_x) break;
                    if(traverse_it->second->Gp!=NULL && traverse_it->second->Gp->x > temp_bound_x ){
                        GP2->Add_edge(traverse_it->second->Gp);
                        
                    }
                    if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                }

                it2->second->Gp = GP2;
            }

            //############check it1  whether exist
            for (traverse_it = it1; traverse_it!=bound_map.end();++traverse_it){
                if(traverse_it->first > all_line_vec[s]->y+all_line_vec[s]->length) break;
                if(traverse_it->first == all_line_vec[s]->y+all_line_vec[s]->length && traverse_it->second->max_x == temp_x ){
                    p1 = true;
                    break;
                }
            }            
            if(p1){
                GP1 = new GraphPoint(all_line_vec[s], UP, G_point_num++);
                temp_clu->Add_GP(GP1);
                //5 construct edge
                temp_bound_x = all_line_vec[s]->S->coords->x1;
                traverse_it = it1;
                ++traverse_it;
                for(;traverse_it!=bound_map.end();++traverse_it){
                    if(temp_bound_x > temp_x) break;
                    if(traverse_it->second->Gp!=NULL && traverse_it->second->Gp->x > temp_bound_x ){
                        GP1->Add_edge(traverse_it->second->Gp);
                        
                    }
                    if(traverse_it->second->max_x > temp_bound_x) temp_bound_x = traverse_it->second->max_x;
                }

                it1->second->Gp = GP1;
            }




    	}

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

    }


    //for (it1 = bound_map.begin();it1 != bound_map.end();++it1)
    //	cout << it1->first << endl;
    cout << "point:" <<G_point_num << endl;
    cout << "shape count:" << Layer_Shape_num << endl;

    check_point_svg();
    //cin.get();

}




void 
Layer::SG_find_GPinfo(int x_pos, BoundLine_info* bl_info){
	

}

























//int convert to string
string itos(int a) {
    string sign = a<0?"-":"";
    string result = a>0?string(1,(a%10+'0')):string(1,((a=-a)%10+'0'));
    (a/=10)>0?result=itos(a)+result:result;
    return sign+result;
 }

void
Layer::check_cluster(){
    //system("mkdir Gnuplot");
    int count = 0;
    for(size_t i = 0; i < all_cluster.size(); i++){
        //cout << i <<":\n";
        count += all_cluster[i]->shape_list.size();


        /*string num = itos(i);
        string a1 = string("Gnuplot/data")+num+string(".txt");
        string gnu = string("Gnuplot/gnu")+num;
        ofstream a(a1.c_str());
        ofstream b(gnu.c_str());

        it_shape it = all_cluster[i]->shape_list.begin();
        //cout<<"elements:"<<endl;
        int idx = 1;
        for(; it != all_cluster[i]->shape_list.end(); it++){
            int x1 = (*it)->coords->x1;
            int x2 = (*it)->coords->x2;
            int y1 = (*it)->coords->y1;
            int y2 = (*it)->coords->y2;
            a<<x1<<" "<<y1<<endl;
            a<<x2<<" "<<y2<<endl;
            b<<"set object " << idx << " rect from "<<x1<<","<<y1<<" to "<<x2<<","<<y2<<" fc lt 2 fs pattern 1 lw 3"<<endl;
            b<<"set label \""<< idx++ <<"\" at "<<(x1+x2)/2<<","<<(y1+y2)/2<<" front center font \",10\""<<endl;
            cout<<(*it)->coords->x1<<", "<<(*it)->coords->y1<<"; "
            <<(*it)->coords->x2<<", "<<(*it)->coords->y2<<endl;

        }
    a<<10000<<" "<<10000<<endl;
    b<<"set arrow from 0,"<<10000<<" to "<<10000<<","<<10000<<" nohead lc 3 lw 5"<<endl;
    b<<"set arrow from "<<10000<<",0 to "<<10000<<","<<10000<<" nohead lc 3 lw 5"<<endl;
    b<<"set term png size 2000,2000"<<endl;
    b<<"set output \'"<<""<<"out"<<num<<".png"<<"\'"<<endl;
    b<<"plot \'"<< a1 <<"\' using 1:2 with points"<<endl;
    a.close();
    b.close();
    string command ="gnuplot "+gnu; 
    system(command.c_str());*/

    }
    cout << "count:" <<count << endl;

}


void 
Layer::check_point_svg(){
    string a1 = string("Point_test")+string(".svg");
    ofstream a(a1.c_str());
    
    a << " <svg xmlns=\"http://www.w3.org/2000/svg\" width=\"7000\" height=\"3000\">\n ";
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
        a << "<rect x=\"" << (*sh_itr)->coords->x1 << "\" y=\"" << (*sh_itr)->coords->y1 << "\" width=\"" << width << "\" height=\""<< height << "\" style=\"fill:blue;stroke:black;stroke-width:1;fill-opacity:0.2;stroke-opacity:0.6\" />" << endl;
    }
    //1.Obstacle shape
    b_itr = Obstacle_list.begin();
    e_itr = Obstacle_list.end();
    for(sh_itr = b_itr; sh_itr!=e_itr;++sh_itr){
        width = (*sh_itr)->coords->x2 - (*sh_itr)->coords->x1;
        height = (*sh_itr)->coords->y2 - (*sh_itr)->coords->y1;
        a << "<rect x=\"" << (*sh_itr)->coords->x1 << "\" y=\"" << (*sh_itr)->coords->y1 << "\" width=\"" << width << "\" height=\""<< height << "\" style=\"fill:gray;stroke:black;stroke-width:1;fill-opacity:0.2;stroke-opacity:0.6\" />" << endl;
    }

    //point
    list < GraphPoint* >::iterator gp_itr,begin_itr,end_itr;
    MAP_GP_edge::iterator map_gp_itr, map_begin_itr, map_end_itr;
    for(size_t i = 0; i < all_cluster.size(); i++){
        begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
            a<< "<circle cx=\"" << (*gp_itr)->x << "\" cy=\""<< (*gp_itr)->y << "\" r=\"2\" style=\"fill:red;stroke:red;stroke-width:3;fill-opacity:0.1;stroke-opacity:0.8\" />" << endl;
            for(map_gp_itr = (*gp_itr)->map_edge.begin();map_gp_itr!=(*gp_itr)->map_edge.end(); ++map_gp_itr){
                a << "<line x1=\"" << (*gp_itr)->x << "\" y1=\"" << (*gp_itr)->y << "\" x2=\"" << map_gp_itr->second->x << "\" y2=\"" << map_gp_itr->second->y << "\"\nstroke-width=\"1\" stroke=\"black\"/>" << endl;
            }
        }
    }

    //edge






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

void Layer::Via_list_append(Via *temp_via){
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
