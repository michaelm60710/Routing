#include "Manager.h"
#include <iostream>
#include <algorithm>

using namespace std;


bool
sort_linex(Line* L1, Line* L2){
    return L1->x < L2->x;
}

void Manager::SpanningGraphConstruct(){
    pair<GraphPoint*, GraphPoint*> GP_result;
	int l_idx=0, temp_layer;
    map< int , BoundLine_info* , less<int> > R_bound_map;
    list < Shape* >::iterator itr1,itr2;
    GraphPoint *gp1, *gp2;
    int x,y, Width = Boundary->x2 - Boundary->x1, Height  = Boundary->y2 - Boundary->y1, temp_min_x;
    size_t clu_end;
    GraphPoint *P1=NULL, *P2=NULL, *P3=NULL, *P4=NULL;
    //bool PP1 = false, PP2 = false, PP3=false, PP4=false;

    //###1. Initialize
    for(int i =0;i<MetalLayers;i++) {
        //cout << "LAYER " << i << ":" << endl;
        all_layer[i].SpanningGraphConstruct();
    }

    //###2. Sort all shape line
	all_line.resize((RoutedShapes+Obstacles+RoutedVias)*2 );

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
        l_l->width = 0;
        l_r->width = l_r->x - l_l->x;
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
    
    //###2.2 
    clu_end = all_line.size() - 1;
    int max_x = Boundary->x2 - Spacing + 1;
    for(size_t s = all_line.size()-1; s > 0; s--){
    	if(all_line[s]->x <= max_x){
    		clu_end = s;
    		break;
    	}
    }

    //###3. Construct global graph
    for(size_t  s = 0; s <= clu_end; s++){
    	temp_layer = all_line[s]->S->layer_position;
    	GP_result = all_layer[temp_layer].SGconstruct(all_line[s]);


        //Init: extra Obs
        int _x = all_line[s]->x, _y2 = all_line[s]->y, _y1 = all_line[s]->y + all_line[s]->length;
        int R_d_y2_len = _y1, R_d_y1_len = _y2, R_u_y2_len = _y1, R_u_y1_len = _y2;
        if(all_line[s]->S->Shape_type!=RSHAPE){
            R_d_y2_len = R_u_y2_len = _y2;
            R_d_y1_len = R_u_y1_len = _y1;
        }


        //int up_layer = MetalLayers-1, down_layer = 0;
        P1 = P2 = P3 = P4 = NULL;
        if(all_line[s]->S->Shape_type==VIA);
        else if(all_line[s]->S->Shape_type==RSHAPE){
            P1 = P2 = P3 = P4 = GP_result.first;
        }
        else{ //OBSTACLE
            P1 = P3 = GP_result.first;
            P2 = P4 = GP_result.second;
        }

        if(all_line[s]->S->Shape_type==RSHAPE && all_line[s]->LR==RIGHT){
            //1. Init R_bound_map
            if(min_x > _x - Width) temp_min_x = min_x;
            else                   temp_min_x = _x - Width;
            R_bound_map.clear();
            R_bound_map.insert(pair< int , BoundLine_info*>(Spacing-1, new BoundLine_info(Width, 0, UP, Spacing-1, temp_min_x, NULL) ) );
            R_bound_map.insert(pair< int , BoundLine_info*>(-1, new BoundLine_info(Width, 0, DOWN, -1, temp_min_x, NULL) ) );
            R_bound_map.insert(pair< int , BoundLine_info*>(Height+1, new BoundLine_info(Width, 0, UP, Height+1, temp_min_x, NULL) ) );
            R_bound_map.insert(pair< int , BoundLine_info*>(Height+1-Spacing, new BoundLine_info(Width, 0, DOWN, Height+1-Spacing, temp_min_x, NULL) ) );
            
            // up down construct edge
            /*if(temp_layer<MetalLayers-1){
                if(P2) P2 = all_layer[temp_layer+1].SGconstruct_extra_obs(_x, _y2, (_y1 - _y2), P2, DOWN, R_u_y2_len);
                if(P1) P1 = all_layer[temp_layer+1].SGconstruct_extra_obs(_x, _y1, (_y1 - _y2), P1, UP, R_u_y1_len);
                all_layer[temp_layer+1].Update_Rbound_map(_x, _y1, _y2, R_bound_map);
            }
            //Up
            for(int i = temp_layer+2;i<=MetalLayers-1;i++) {
                all_layer[i].Extra_obs_RSHAPE_right(all_line[s], P1, P2, _x, _y1, _y2, R_u_y1_len, R_u_y2_len, R_bound_map);
                if(P1==NULL && P2==NULL) break;
                all_layer[i].Update_Rbound_map(_x, _y1, _y2, R_bound_map);
            }

            R_bound_map.clear();
            R_bound_map.insert(pair< int , BoundLine_info*>(Spacing-1, new BoundLine_info(Width, 0, UP, Spacing-1, temp_min_x, NULL) ) );
            R_bound_map.insert(pair< int , BoundLine_info*>(-1, new BoundLine_info(Width, 0, DOWN, -1, temp_min_x, NULL) ) );
            R_bound_map.insert(pair< int , BoundLine_info*>(Height+1, new BoundLine_info(Width, 0, UP, Height+1, temp_min_x, NULL) ) );
            R_bound_map.insert(pair< int , BoundLine_info*>(Height+1-Spacing, new BoundLine_info(Width, 0, DOWN, Height+1-Spacing, temp_min_x, NULL) ) );
           */

            if(temp_layer>0){
                if(P4) P4 = all_layer[temp_layer-1].SGconstruct_extra_obs(_x, _y2, (_y1 - _y2), P4, DOWN, R_d_y2_len);
                if(P3) P3 = all_layer[temp_layer-1].SGconstruct_extra_obs(_x, _y1, (_y1 - _y2), P3, UP, R_d_y1_len);
                all_layer[temp_layer-1].Update_Rbound_map(_x, _y1, _y2, R_bound_map);
            }           
            //Down
            for(int i = temp_layer-2; i>=0;i--) {
                all_layer[i].Extra_obs_RSHAPE_right(all_line[s], P3, P4, _x, _y1, _y2, R_d_y1_len, R_d_y2_len, R_bound_map);
                if(P3==NULL && P4==NULL) break;
                all_layer[i].Update_Rbound_map(_x, _y1, _y2, R_bound_map);
            }

        }
        else{
            // up down construct edge
            if(temp_layer<MetalLayers-1){
                if(P2) P2 = all_layer[temp_layer+1].SGconstruct_extra_obs(_x, _y2, (_y1 - _y2), P2, DOWN, R_u_y2_len);
                if(P1) P1 = all_layer[temp_layer+1].SGconstruct_extra_obs(_x, _y1, (_y1 - _y2), P1, UP, R_u_y1_len);
            }
            if(temp_layer>0){
                if(P4) P4 = all_layer[temp_layer-1].SGconstruct_extra_obs(_x, _y2, (_y1 - _y2), P4, DOWN, R_d_y2_len);
                if(P3) P3 = all_layer[temp_layer-1].SGconstruct_extra_obs(_x, _y1, (_y1 - _y2), P3, UP, R_d_y1_len);
            }


            //Up
            for(int i = temp_layer+2;i<=MetalLayers-1;i++) {
                all_layer[i].Extra_obs(all_line[s], P1, P2, _x, _y1, _y2, R_u_y1_len, R_u_y2_len);
                if(P1==NULL && P2==NULL) break;
            }
            //Down
            for(int i = temp_layer-2; i>=0;i--) {
                all_layer[i].Extra_obs(all_line[s], P3, P4, _x, _y1, _y2, R_d_y1_len, R_d_y2_len);
                if(P3==NULL && P4==NULL) break;
            }

        }

    }

    //###4. Update Via length (Cluster of Via must be only 1 Gp)
    for(int i=0;i<MetalLayers-1;i++){
        itr1 = all_layer[i].Via_list.begin();
        itr2 = all_layer[i+1].Upper_Via_list.begin();
        while(itr1!=all_layer[i].Via_list.end()){
            x = (*itr1)->coords->x1;
            y = (*itr1)->coords->y1;
            gp1 = *((*itr1)->clu->GraphP_list.begin());
            gp2 = *((*itr2)->clu->GraphP_list.begin());
            gp1->Add_edge(gp2, x, y, x, y,i, 0);// bug: need to check gp1 & gp2 aren't NULL
            ++itr1;
            ++itr2;
        }
    }

    //###5. Convert to undirected graph
    for(int i =0;i<MetalLayers;i++) all_layer[i].ConvertToUndirectedG();
    
}

void Manager::SpanningTreeConstruct(){
    
    //###1. Init all_cluster
    size_t all_cluster_size = 0;
    for(int i =0;i<MetalLayers;i++) all_cluster_size += all_layer[i].all_cluster.size();
    all_cluster.reserve(all_cluster_size);
    for(int i =0;i<MetalLayers;i++) all_cluster.insert(all_cluster.end(), all_layer[i].all_cluster.begin(), all_layer[i].all_cluster.end());

    //###2. Extended Dijkstra's Algorithm
    ExtendedDijkstra();

    //###3. Extended Kruskal's Algorithm
    ExtendedKruskal();

    //###4. Final GP list construct
    GraphPoint *r_gp;
    for(size_t i = 0; i < all_cluster.size(); i++){
    	if(all_cluster[i]->GetShapeType()==OBSTACLE) continue;
    	r_gp = *(all_cluster[i]->GraphP_list.begin());
    	if(r_gp->select) continue;
    	r_gp->select = true;
    	gp_list.push_back(r_gp);
    }



}

void Manager::ExtendedDijkstra(){
	//cout << "...Start Dijkstra...\n";
    FibHeap<int> FibH;// FibH;
    FibHeap<int>::FibNode *temp_fibn;
    list < GraphPoint* >::iterator gp_itr,begin_itr,end_itr;
    MAP_GP_edge::iterator map_gp_itr;
    GraphPoint *temp_gp, *temp_gp2;
    int temp_dis, temp_dis2;


    //### 1. insert all GP in fibo heap & initialize SET
    for(size_t i = 0; i < all_cluster.size(); i++){
        begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        if(all_cluster[i]->GetShapeType()==RSHAPE || all_cluster[i]->GetShapeType()==VIA){
            (*begin_itr)->parent = (*begin_itr); //SET root
            (*begin_itr)->terminal_dis = 0;
            (*begin_itr)->Fnode = FibH.push(0, (*begin_itr));
            (*begin_itr)->select = false;
        }
        else{
            for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
                (*gp_itr)->parent = NULL;
                (*gp_itr)->terminal_dis = INT_MAX;
                (*gp_itr)->Fnode = FibH.push(INT_MAX, (*gp_itr));
                (*gp_itr)->select = false;
            }
        }
    }

    //### 2. Shortest path terminal forest construct
    while(!FibH.empty()){
        temp_fibn = FibH.topNode();
        temp_dis = temp_fibn->key;
        temp_gp = (GraphPoint*)temp_fibn->payload;
        temp_gp->select = true;

        //# pop the target vertex
        FibH.pop();

        //# update edge   // map_gp_itr->second->Gp .... not good
        for(map_gp_itr = temp_gp->map_edge.begin();map_gp_itr!=temp_gp->map_edge.end(); ++map_gp_itr){
            temp_gp2 = map_gp_itr->second->Gp;
            temp_dis2 = map_gp_itr->second->distance + temp_dis;
            if( temp_gp2->select==false && temp_gp2->terminal_dis > temp_dis2 ){
                temp_gp2->terminal_dis = temp_dis2;
                temp_gp2->parent = temp_gp;

                FibH.decrease_key(temp_gp2->Fnode, temp_dis2);
            }

        }

    }

    //### 3. Find Set
    int num_vertex=0, isolate_obstacle_num=0, isolate_num=0;
    for(size_t i = 0; i < all_cluster.size(); i++){
        begin_itr = all_cluster[i]->GraphP_list.begin();
        end_itr = all_cluster[i]->GraphP_list.end();
        for(gp_itr = begin_itr; gp_itr!=end_itr;++gp_itr){
             (*gp_itr)->root = (*gp_itr)->Find_Set();
             num_vertex++;
             //debug
             if((*gp_itr)->root==NULL){
             	isolate_num++;
                string sshape;
                if((*gp_itr)->Shape_type==RSHAPE)   sshape = "RSHAPE";
                else if((*gp_itr)->Shape_type==VIA) sshape = "VIA";
                else                                {
                	sshape = "OBSTACLE"; 
                	isolate_obstacle_num++;
                }
                //cerr << "A " << sshape << "'s vertex is isolated ";
                for(map_gp_itr = (*gp_itr)->map_edge.begin();map_gp_itr!=(*gp_itr)->map_edge.end(); ++map_gp_itr){
                    //cout << "a"<< (*gp_itr)->idx;
                    //cout <<"\n" <<  map_gp_itr->second->distance << endl;
                }
                //cout << endl;
             }
        }
    }
    if(isolate_num!=0){
    	//cerr << isolate_num << " vertex are isolated\n";
    	//cerr << isolate_obstacle_num << " OBSTACLE vertex are isolated\n";
    }

    //#check
    //cout << "num vertex: " << num_vertex << endl;


}

void Manager::ExtendedKruskal() {
    //cout << "...Start Kruskal's...\n";
    list <GraphPoint*>::iterator gp_itr, begin1, end1;
    MAP_GP_edge::iterator map_gp_itr, begin2, end2;
    GraphPoint *temp_gp1, *temp_gp2;
    int edgeLength, dis1, dis2;

    multimap < int, Edge_info* > HeapBE; // bridge edge
    multimap < int, Edge_info* >::iterator curEdge;

    // initialization
    int SET_count= 0;
    for (size_t i = 0; i < all_cluster.size(); i++) {
        begin1 = all_cluster[i]->GraphP_list.begin();
        end1 = all_cluster[i]->GraphP_list.end();
        if(all_cluster[i]->GetShapeType()==RSHAPE || all_cluster[i]->GetShapeType()==VIA){
             (*(all_cluster[i]->GraphP_list.begin() ))->parentKK = (*(all_cluster[i]->GraphP_list.begin() ));
             SET_count++;
        }
        for (gp_itr = begin1; gp_itr != end1; ++gp_itr) {
            temp_gp1 = (*gp_itr);
            temp_gp1->select = false; // final edge need to init
            temp_gp1->parentKK = temp_gp1;
            temp_gp1->visit = true;
            begin2 = temp_gp1->map_edge.begin();
            end2 = temp_gp1->map_edge.end();

            for (map_gp_itr = begin2; map_gp_itr != end2; ++map_gp_itr) {
                temp_gp2 = map_gp_itr->second->Gp;
                if (temp_gp2->visit == true) continue;
                if (temp_gp1->root != temp_gp2->root) {
                	edgeLength = map_gp_itr->second->distance;
	                dis1 = temp_gp1->terminal_dis;
	                dis2 = temp_gp2->terminal_dis;
                    map_gp_itr->second->source = temp_gp1;
                    HeapBE.insert(  pair <int, Edge_info*> ( dis1 + edgeLength + dis2, map_gp_itr->second )  );
                }
            }
        }
    }

    // choose MST points
    int set_combine = 1;
    for(curEdge = HeapBE.begin(); curEdge != HeapBE.end(); ++curEdge){
        temp_gp1 = curEdge->second->source;
        temp_gp2 = curEdge->second->Gp;
        GraphPoint *set1 = findSet(temp_gp1->root);
        GraphPoint *set2 = findSet(temp_gp2->root);

        if (set1 != set2) {
            set_combine++;
            unionSet(set1, set2);
            addMSTEdges(temp_gp1, temp_gp2, false);
            if(set_combine>=SET_count) {
            cout << "longest path= " << curEdge->first << endl;
            break;
        }
        }
        

    }
}



GraphPoint* Manager::findSet(GraphPoint *p) {
    if (p != p->parentKK)
        p->parentKK = findSet(p->parentKK);
    return p->parentKK;
}

void Manager::unionSet( GraphPoint *s1, GraphPoint *s2 ) {

    if (s1->rank >= s2->rank)
        s2->parentKK = s1;
    else 
        s1->parentKK = s2;

    if (s1->rank == s2->rank)
        s1->rank++;

}

void Manager::addMSTEdges(GraphPoint *p1, GraphPoint *p2, bool opt1) { //need to optimize! use map search
    GraphPoint *p = p1;
    //test 
    if(opt1 && p1->root->Layer_pos!=p2->root->Layer_pos && !(p1->Shape_type!=OBSTACLE && p2->Shape_type!=OBSTACLE) ) { //bug: roots are same layer
        Optimize1(p1, p2);
        return;
    }
    //else return;

    MAP_GP_edge::iterator map_gp_itr, map_begin_itr, map_end_itr;
    add_Final_GP(p1,p2, false);

    while (p != p->parent) {
    	if(!p->select){
    		add_Final_GP(p,p->parent, true);
    		p->select = true;
    	}
    	else break;

        p = p->parent;
    }
    p = p2;
    while (p != p->parent) {
    	if(!p->select){
    		add_Final_GP(p,p->parent, true);
    		p->select = true;
    	}
    	else break;

        p = p->parent;
    }

}

void Manager::add_Final_GP(GraphPoint *p1, GraphPoint *p2, bool insert_gp_list) {
	MAP_GP_edge::iterator it1 = p1->map_edge.find(p2->idx);
	Edge_info* temp_edge;
	Edge_info *E1;
	if(it1 == p1->map_edge.end() ) cout << insert_gp_list << "??\n";
	else {
		if(insert_gp_list) gp_list.push_back(p1);
		temp_edge = it1->second;
		p1->final_edge.push_back(temp_edge);
		E1 = new Edge_info(p1, temp_edge->point_x2, temp_edge->point_y2, temp_edge->point_x1, temp_edge->point_y1, temp_edge->distance, temp_edge->layer);
		p2->final_edge.push_back(E1);
	}

}

void Manager::Optimize1(GraphPoint *p1, GraphPoint* p2){
	GraphPoint *temp_p, *temp_p1, *p_begin, *p_end, *insert_gp;	
    MAP_GP_edge::iterator it1;
    list<Edge_info*>::iterator edge_itr;
    Edge_info *temp_edge, *temp_edge1, *E1;
    FibHeap<int> FibH;// FibH;
    FibHeap<int>::FibNode *temp_fibn;
    int temp_layer;
    int x1,y1,x2,y2, xx1, yy1, xx2, yy2 ,bound_x1, bound_x2, bound_y1, bound_y2, new_x, new_y, dis;
    int nearest_point;
    //  2------3
    //  0------1

	//ftemp_edge
    p_begin = p1->root;
    p_end = p2->root;

    //###1.1 construct path & init ftemp_edge
    Recur_parent_opt1(p1);
    p1->path = temp_p = p2;
    while(temp_p != temp_p->parent){
        temp_p->path = temp_p->parent;
        temp_p = temp_p->parent;
    }

    //###1.2 find ftemp_edge
    p_end->terminal_dis = INT_MAX;
    p_end->Fnode = FibH.push(INT_MAX, temp_p);
    p_end->ftemp_edge.clear();
    for(temp_p = p_begin;temp_p != p_end; temp_p = temp_p->path){
        it1 = temp_p->map_edge.find(temp_p->path->idx);
        temp_p->ftemp_edge.clear();
        temp_p->ftemp_edge.push_back(it1->second);
        temp_p->terminal_dis = INT_MAX;
        temp_p->Fnode = FibH.push(INT_MAX, temp_p);
    }
    //###1.3 construct new overlap via
    temp_layer = (*(p_begin->ftemp_edge.begin()))->layer;
    GraphPoint *same_l_gp = p_begin;
    for(temp_p = p_begin;temp_p != p_end; temp_p = temp_p->path){
    	temp_edge = *(temp_p->ftemp_edge.begin()); 
    	if(temp_layer!=temp_edge->layer){
            if(abs(temp_layer-temp_edge->layer)>1);
            else{
                opt1_shape(temp_edge,x1,y1,x2,y2);//cout << "x1: " << temp_edge->point_x1 << ", y1: " <<temp_edge->point_y1 << ", x2:" << temp_edge->point_x2 << ", y2: "<< temp_edge->point_y2 << ", pos:" << nearest_point << endl;
                for(temp_p1 = same_l_gp; temp_p1!=temp_p; temp_p1 = temp_p1->path){//main part: consider overlap
                    temp_edge1 =  *(temp_p1->ftemp_edge.begin()); 
                    nearest_point = opt1_shape(temp_edge1,xx1,yy1,xx2,yy2);
                    
                    //###Find overlap bound begin
                    if(xx1 < x1){
                        bound_x1 = x1;
                        if(xx2 < x1) continue;
                        else if(xx2 > x2) bound_x2 = x2;                        
                        else              bound_x2 = xx2;
                    }
                    else if(xx1 > x2) continue;
                    else{//x1 <= xx1 <= x2
                        bound_x1 = xx1;
                        if(xx2 > x2) bound_x2 = x2;
                        else         bound_x2 = xx2;
                    }

                    if(yy1 < y1){
                        bound_y1 = y1;
                        if(yy2 < y1) continue;
                        else if(yy2 > y2) bound_y2 = y2;                        
                        else              bound_y2 = yy2;
                    }
                    else if(yy1 > y2) continue;
                    else{//y1 <= yy1 <= y2
                        bound_y1 = yy1;
                        if(yy2 > y2) bound_y2 = y2;
                        else         bound_y2 = yy2;
                    }
                    if(bound_x1==bound_x2 && bound_y1==bound_y2) continue;
                    if(nearest_point==0){
                        new_x = bound_x1;
                        new_y = bound_y1;
                    }    
                    else if(nearest_point==1){
                        new_x = bound_x2;
                        new_y = bound_y1;
                    }
                    else if(nearest_point==2){
                        new_x = bound_x1;
                        new_y = bound_y2;
                    }
                    else if(nearest_point==3){
                        new_x = bound_x2;
                        new_y = bound_y2;
                    }
                    //###Insert 2 edge
                    insert_gp = new GraphPoint(temp_layer, new_x, new_y);
                    insert_gp->Fnode = FibH.push(INT_MAX, insert_gp);
                    dis = abs(temp_edge->point_x2 - new_x) + abs(temp_edge->point_y2 - new_y);
                    E1 = new Edge_info(temp_edge->Gp, new_x, new_y, temp_edge->point_x2, temp_edge->point_y2, dis, temp_edge->layer);
                    insert_gp->ftemp_edge.push_back(E1);
                    dis = abs(temp_edge1->point_x1 - new_x) + abs(temp_edge1->point_y1 - new_y);
                    E1 = new Edge_info(insert_gp, temp_edge1->point_x1, temp_edge1->point_y1 ,new_x, new_y, dis, temp_edge1->layer);
                    temp_p1->ftemp_edge.push_back(E1);
                }
            }
    		
    		temp_layer = temp_edge->layer;
    		same_l_gp = temp_p;
    	}
    }

    //###1.4 find shortest path (dijkstra's)
    FibH.decrease_key(p_begin->Fnode, 0);
    p_begin->terminal_dis = 0;
    while(!FibH.empty()){
        temp_fibn = FibH.topNode();
        temp_p = (GraphPoint*)temp_fibn->payload;
        if(temp_p==p_end) break;
        FibH.pop();

        for(edge_itr = temp_p->ftemp_edge.begin(); edge_itr != temp_p->ftemp_edge.end(); ++edge_itr){
            dis = temp_p->terminal_dis + (*edge_itr)->distance;
            temp_p1 = (*edge_itr)->Gp;
            if(temp_p1->terminal_dis > dis){
            	temp_p1->terminal_dis = dis;
            	temp_p1->path_opt = temp_p;
            	FibH.decrease_key(temp_p1->Fnode, dis);
            }
        }

    }

    //###2 construct the final edge
    for(temp_p1 = p_end; temp_p1 != p_begin; temp_p1 = temp_p1->path_opt){ ////bug
    	temp_edge = NULL;
    	temp_p = temp_p1->path_opt;
        for(edge_itr = temp_p->ftemp_edge.begin(); edge_itr != temp_p->ftemp_edge.end(); ++edge_itr){
            if((*edge_itr)->Gp==temp_p1){
                temp_edge = (*edge_itr);
                break;
            }
        }
        if(temp_edge==NULL) { cerr << "error!"<< endl; continue;}//bug
        
        temp_p->final_edge.push_back(temp_edge);
        E1 = new Edge_info(temp_p, temp_edge->point_x2, temp_edge->point_y2, temp_edge->point_x1, temp_edge->point_y1, temp_edge->distance, temp_edge->layer);
        temp_p1->final_edge.push_back(E1);

    }

    //###2.1 add to graph list
    for(temp_p = p_end;temp_p != p_begin; temp_p = temp_p->path_opt){
        if(!temp_p->select){
            gp_list.push_back(temp_p);
            temp_p->select = true;
        }
    }



}

void Manager::Recur_parent_opt1(GraphPoint* gp1){


	if(gp1 != gp1->parent) {
		if(gp1->parent==NULL){
			cout << "errorQQ" << endl;
			//cin.get();
		}
		gp1->parent->path = gp1;
		Recur_parent_opt1(gp1->parent);
	}

}

int Manager::opt1_shape(Edge_info *E, int &x1, int &y1, int &x2, int &y2){
	int nearest_pos = 0;
	x1 = E->point_x1;
	x2 = E->point_x2;
	y1 = E->point_y1;
	y2 = E->point_y2;
	if(x1 > x2){
		swap(x1, x2);
		nearest_pos++;
	}
	if(y1 > y2){
		swap(y1, y2);
		nearest_pos+=2;
	}

	return nearest_pos;
}

















string Manager::itos1(int a) {
    string sign = a<0?"-":"";
    string result = a>0?string(1,(a%10+'0')):string(1,((a=-a)%10+'0'));
    (a/=10)>0?result=itos1(a)+result:result;
    return (sign+result);
 }

