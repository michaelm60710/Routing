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
        l_l->LR = RIGHT;
        //cout << "x,y,x2,y2:"<< all_shape_vec[s]->coords->x1 << " " << all_shape_vec[s]->coords->y1 << " " << all_shape_vec[s]->coords->x2 << " " <<all_shape_vec[s]->coords->y2 << endl;
        //cout << "l_l:" << l_l->x << " " << l_l->y << " " << l_l->length << " " <<  l_l->LR << endl;
        //cout << "l_r:" << l_r->x << " " << l_r->y << " " << l_r->length << " " <<  l_r->LR << endl;
        all_line_vec[2*s] = l_l;
        all_line_vec[2*s+1] = l_r;
    }
    sort(all_line_vec.begin(), all_line_vec.end(), sort_line_x);

    //### 2. lets go~
    multimap< int , BoundLine_info* , less<int> > bound_map;
    multimap< int , BoundLine_info* , less<int> >::iterator it1,it2;
    bool t_shape_type,p1,p2;
    for(size_t s = 0; s < all_line_vec.size(); s++){
    	t_shape_type = all_line_vec[s]->S->Shape_type;

    	if(t_shape_type==RSHAPE){

    	}
    	else{ //t_shape_type==OBSTACLE

    	}


    	if(all_line_vec[s]->LR==LEFT){ //LEFT
    		BoundLine_info* b1 = new BoundLine_info{NULL, all_line_vec[s]->LR, all_line_vec[s]->S->coords->x2};
    		BoundLine_info* b2 = new BoundLine_info{NULL, all_line_vec[s]->LR, all_line_vec[s]->S->coords->x2};
    		it1 = bound_map.insert(pair< int , BoundLine_info*>(all_line_vec[s]->y+all_line_vec[s]->length, b1) );
    		it2 = bound_map.insert(pair< int , BoundLine_info*>(all_line_vec[s]->y                        , b2) );
    	}
    	else{ //RIGHT

    	}


    }



    //for (it1 = bound_map.begin();it1 != bound_map.end();++it1)
    //	cout << it1->first << endl;
    cin.get();

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
