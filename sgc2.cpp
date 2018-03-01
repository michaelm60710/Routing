#include "Manager.h"
#include <iostream>
#include <algorithm>
#include <queue>  

using namespace std;

bool
sort_gp_x(GraphPoint* G1, GraphPoint* G2){
    return G1->x < G2->x;
}

void Manager::SpanningGraphConstruct_2(){
	vector < Line*  >::iterator clu_end, clu_itr;
	vector < GraphPoint*  >::iterator gp_itr;
	list < Shape* >::iterator itr1,itr2;
	GraphPoint *gp1, *gp2;
	int temp_x = 0, temp_layer;
	int x,y;
	GraphPoint *gp_point;
    //###1. Initialize
    for(int i =0;i<MetalLayers;i++) {
        all_layer[i].SpanningGraphConstruct_2();
    }

    //###2. Sort gp_vec
    sort(gp_vec.begin(), gp_vec.end(), sort_gp_x);
    
    //###2.2 
    clu_itr = --all_line.end();
    int max_x = Boundary->x2 - Spacing + 1;
    for(clu_itr = --all_line.end(); clu_itr != all_line.begin(); --clu_itr){
    	if((*clu_itr)->x <= max_x){
    		clu_end = clu_itr;
    		++clu_end;
    		break;
    	}
    }

    //###3. Construct global graph
    gp_itr = gp_vec.begin();
    for(clu_itr = all_line.begin(); clu_itr != clu_end; ++clu_itr ){
    	temp_x = (*clu_itr)->x;
    	while( gp_itr != gp_vec.end() && (*gp_itr)->x <= temp_x ){
    		temp_layer = (*gp_itr)->Layer_pos;
    		gp_point = all_layer[temp_layer].SGconstruct_2(NULL, (*gp_itr), false);
    		// up down construct edge 
	        if(temp_layer>0)
	            all_layer[temp_layer-1].SGcons_RshapeOverlap_2(gp_point);
	        if(temp_layer<MetalLayers-1)
	            all_layer[temp_layer+1].SGcons_RshapeOverlap_2(gp_point);

    		++gp_itr;
    	}
    	temp_layer = (*clu_itr)->S->layer_position;
    	all_layer[temp_layer].SGconstruct_2((*clu_itr), NULL, true);
    }



    //#### all_gp_vec
    int gp_num = gp_vec.size();
    for(vector <Cluster*>::iterator c_itr = all_cluster.begin(); c_itr != all_cluster.end(); ++c_itr){
    	if((*c_itr)->GetShapeType()!=OBSTACLE) gp_num++;
    }
    all_gp_vec.reserve(gp_num);
    all_gp_vec.insert(all_gp_vec.end(), gp_vec.begin(), gp_vec.end());
    all_gp_rshape_begin = all_gp_vec.end();
    for(vector <Cluster*>::iterator c_itr = all_cluster.begin(); c_itr != all_cluster.end(); ++c_itr)
    	if((*c_itr)->GetShapeType()!=OBSTACLE) all_gp_vec.push_back(*((*c_itr)->GraphP_list.begin()));


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
    pair<MAP_GP_edge::iterator, bool> map_gp_status;
    MAP_GP_edge::iterator map_gp_itr;
    GraphPoint *temp_gp;
    Edge_info *E1 = NULL;
    int x1,y1,x2,y2,distance;
    for(vector <GraphPoint*>::iterator gpv_itr = all_gp_vec.begin(); gpv_itr != all_gp_vec.end(); ++gpv_itr){
    	for(map_gp_itr = (*gpv_itr)->map_edge.begin();map_gp_itr!=(*gpv_itr)->map_edge.end(); ++map_gp_itr){
        	temp_gp = map_gp_itr->second->Gp;
            x1 = map_gp_itr->second->point_x1;
            y1 = map_gp_itr->second->point_y1;
            x2 = map_gp_itr->second->point_x2;
            y2 = map_gp_itr->second->point_y2;
            distance = map_gp_itr->second->distance;
        	map_gp_status = temp_gp->map_edge.insert(pair< int , Edge_info*>((*gpv_itr)->idx, E1) );
        	if(map_gp_status.second==true){
				E1 = new Edge_info((*gpv_itr), x2, y2, x1, y1, distance, map_gp_itr->second->layer);
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

void Manager::SpanningTreeConstruct_2(){
	
	//###1. Init final edge
	gp_list.clear();
	for(vector <GraphPoint*>::iterator gpv_itr = all_gp_vec.begin(); gpv_itr != all_gp_vec.end(); ++gpv_itr){
		(*gpv_itr)->final_edge.clear();
		(*gpv_itr)->ftemp_edge.clear();
		(*gpv_itr)->path = NULL;
		(*gpv_itr)->path_opt = NULL;
		(*gpv_itr)->visit = false;
		(*gpv_itr)->rank = 0;
	}

	//###2. Extended Dijkstra's Algorithm
    ExtendedDijkstra_2();
    //###3. Extended Kruskal's Algorithm
    ExtendedKruskal_2();
    //ExtendedPrims();
    
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

void Manager::ExtendedDijkstra_2(){
	cout << "...Start Dijkstra_2...\n";
    FibHeap<int> FibH;// FibH;
    FibHeap<int>::FibNode *temp_fibn;
    list < GraphPoint* >::iterator gp_itr,begin_itr,end_itr;
    MAP_GP_edge::iterator map_gp_itr;
    GraphPoint *temp_gp, *temp_gp2;
    int temp_dis, temp_dis2;


    //### 1. insert all GP in fibo heap & initialize SET
    for(vector <GraphPoint*>::iterator gpv_itr = all_gp_vec.begin(); gpv_itr != all_gp_vec.end(); ++gpv_itr){
    	//cout << (*gpv_itr)->Shape_type << ": " << (*gpv_itr)->map_edge.size() << endl;
        (*gpv_itr)->select = false;
    	if((*gpv_itr)->Shape_type==RSHAPE || (*gpv_itr)->Shape_type==VIA){
    		(*gpv_itr)->parent = (*gpv_itr); //SET root
            (*gpv_itr)->terminal_dis = 0;
            (*gpv_itr)->Fnode = FibH.push(0, (*gpv_itr));
    	}
    	else{//OBSTACLE gp
			(*gpv_itr)->parent = NULL;
            (*gpv_itr)->terminal_dis = INT_MAX;
            (*gpv_itr)->Fnode = FibH.push(INT_MAX, (*gpv_itr));
		}
    }

    //### 2. Shortest path terminal forest construct
    while(!FibH.empty()){
        temp_fibn = FibH.topNode();
        temp_dis = temp_fibn->key;
        temp_gp = (GraphPoint*)temp_fibn->payload;
        temp_gp->select = true;
        //cout << "Top: " << temp_dis << endl;

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
    for(vector <GraphPoint*>::iterator gpv_itr = all_gp_vec.begin(); gpv_itr != all_gp_vec.end(); ++gpv_itr){
    	(*gpv_itr)->root = (*gpv_itr)->Find_Set();
	    num_vertex++;
	    //debug
	    if((*gpv_itr)->root==NULL){
	     	isolate_num++;
	        string sshape;
	        if((*gpv_itr)->Shape_type==RSHAPE)   sshape = "RSHAPE";
	        else if((*gpv_itr)->Shape_type==VIA) sshape = "VIA";
	        else                                {
	        	sshape = "OBSTACLE"; 
	        	isolate_obstacle_num++;
	        }
	        //cerr << "A " << sshape << "'s vertex is isolated ";
	        for(map_gp_itr = (*gpv_itr)->map_edge.begin();map_gp_itr!=(*gpv_itr)->map_edge.end(); ++map_gp_itr){
	            //cout << "a"<< (*gpv_itr)->idx;
	            //cout <<"\n" <<  map_gp_itr->second->distance << endl;
	        }
	        //cout << endl;
	    }
    }
    if(isolate_num!=0){
    	cerr << isolate_num << " vertex are isolated\n";
    	cerr << isolate_obstacle_num << " OBSTACLE vertex are isolated\n";
    }

    //#check
    cout << "num vertex: " << num_vertex << endl;


} 

void Manager::ExtendedKruskal_2() {
    cout << "...Start Kruskal's _2...\n";
    list <GraphPoint*>::iterator gp_itr, begin1, end1;
    MAP_GP_edge::iterator map_gp_itr, begin2, end2;
    GraphPoint *temp_gp1, *temp_gp2;
    int edgeLength, dis1, dis2;

    multimap < int, Edge_info* > HeapBE; // bridge edge
    multimap < int, Edge_info* >::iterator curEdge;

    // initialization
    int ttt= 0;
    for(vector <GraphPoint*>::iterator gpv_itr = all_gp_vec.begin(); gpv_itr != all_gp_vec.end(); ++gpv_itr){
    	if((*gpv_itr)->Shape_type==RSHAPE || (*gpv_itr)->Shape_type==VIA){
             (*gpv_itr)->parentKK = (*gpv_itr);
             ttt++;
        }
        temp_gp1 = (*gpv_itr);
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

    // choose MST points
    for(curEdge = HeapBE.begin(); curEdge != HeapBE.end(); ++curEdge){
    	temp_gp1 = curEdge->second->source;
        temp_gp2 = curEdge->second->Gp;
        GraphPoint *set1 = findSet(temp_gp1->root);
        GraphPoint *set2 = findSet(temp_gp2->root);
        if (set1 != set2) {
            unionSet(set1, set2);
            addMSTEdges(temp_gp1, temp_gp2, true);
        }

    }

}

void Manager::ExtendedPrims(){
    cout << "...Start extend Prim's ...\n";
    FibHeap<int> FibH;// FibH;
    FibHeap<int>::FibNode *temp_fibn;
    queue<GraphPoint*> myqueue;
    list < GraphPoint* >::iterator gp_itr,begin_itr,end_itr;
    MAP_GP_edge::iterator map_gp_itr;
    GraphPoint *temp_gp, *temp_gp2, *prims_temp_gp;
    int edgeLength;

    //###1. construct tree node && Init 
    for(vector <GraphPoint*>::iterator gpv_itr = all_gp_vec.begin(); gpv_itr != all_gp_vec.end(); ++gpv_itr){
        if((*gpv_itr)->parent && (*gpv_itr)->parent!=(*gpv_itr) ) (*gpv_itr)->parent->prims_tree_nd.push_back((*gpv_itr));
        (*gpv_itr)->prims_select = false;
        (*gpv_itr)->prims_weight = INT_MAX;
        (*gpv_itr)->Fnode = NULL;
        (*gpv_itr)->select = false; // final edge need to init
    }

    //###2-1. select first vertex and update the edge 
    (*all_gp_rshape_begin)->prims_weight = 0;
    myqueue.push(*all_gp_rshape_begin);
    while(!myqueue.empty()){
        temp_gp = myqueue.front();
        myqueue.pop();
        for(list<GraphPoint*>::iterator g_itr = temp_gp->prims_tree_nd.begin(); g_itr != temp_gp->prims_tree_nd.end(); ++g_itr) myqueue.push(*g_itr);
        for (map_gp_itr = temp_gp->map_edge.begin(); map_gp_itr != temp_gp->map_edge.end(); ++map_gp_itr) {
            temp_gp2 = map_gp_itr->second->Gp;
            if (temp_gp->root != temp_gp2->root) {
                edgeLength = map_gp_itr->second->distance + temp_gp->terminal_dis + temp_gp2->terminal_dis;
                if(temp_gp2->root->prims_weight > edgeLength) {
                	temp_gp2->root->prims_weight = edgeLength;
                	map_gp_itr->second->source = temp_gp;
                	temp_gp2->root->prims_edge = map_gp_itr->second;
                }
            }
        }

    }

    //###2-2. Insert Vertex
    for(vector <GraphPoint*>::iterator gpv_itr = ++all_gp_rshape_begin; gpv_itr != all_gp_vec.end(); ++gpv_itr){
    	if((*gpv_itr)->Shape_type!=OBSTACLE) (*gpv_itr)->Fnode = FibH.push((*gpv_itr)->prims_weight, (*gpv_itr));
    }

    //###2-3. Extended Prim's Algorithm
    while(!FibH.empty()){
        temp_fibn = FibH.topNode();
        edgeLength = temp_fibn->key;
        prims_temp_gp = (GraphPoint*)temp_fibn->payload;
        prims_temp_gp->prims_select = true;
        //cerr << "Top: " << edgeLength << endl;

        //# pop the target vertex
        FibH.pop();

        //# update edge (new gp)
        while(!myqueue.empty()) myqueue.pop();
        if(prims_temp_gp->root != prims_temp_gp) cout << "wrong?";
        prims_temp_gp->prims_weight = 0;
        myqueue.push(prims_temp_gp);
        while(!myqueue.empty()){
	        temp_gp = myqueue.front();
	        myqueue.pop();
	        for(list<GraphPoint*>::iterator g_itr = temp_gp->prims_tree_nd.begin(); g_itr != temp_gp->prims_tree_nd.end(); ++g_itr) myqueue.push(*g_itr);
	        for (map_gp_itr = temp_gp->map_edge.begin(); map_gp_itr != temp_gp->map_edge.end(); ++map_gp_itr) {
	            temp_gp2 = map_gp_itr->second->Gp;
	            if(temp_gp2->root->prims_select) continue;
	            if (temp_gp->root != temp_gp2->root) {
	                edgeLength = map_gp_itr->second->distance + temp_gp->terminal_dis + temp_gp2->terminal_dis;
	                if(temp_gp2->root->prims_weight > edgeLength) {
	                	temp_gp2->root->prims_weight = edgeLength;
	                	map_gp_itr->second->source = temp_gp;
	                	temp_gp2->root->prims_edge = map_gp_itr->second;
	                	FibH.decrease_key(temp_gp2->root->Fnode, edgeLength);
	                }
	            }
	        }
    	}
    	//# update edge (new path)
    	update_path(prims_temp_gp->prims_edge->source, FibH);
    	update_path(prims_temp_gp->prims_edge->Gp, FibH);

       

        //# insert MST edge
        if(prims_temp_gp->prims_edge)
        	addMSTEdges(prims_temp_gp->prims_edge->source, prims_temp_gp->prims_edge->Gp, true);
       	else{
       		cout << "bug?";
       	}

    }

}

void Manager::update_path(GraphPoint *p, FibHeap<int> &FibH){
	MAP_GP_edge::iterator map_gp_itr;
	GraphPoint *temp_gp2;

	while (p != p->parent) {
    	if(p->terminal_dis!=0){
    		p->terminal_dis = 0;
    		for (map_gp_itr = p->map_edge.begin(); map_gp_itr != p->map_edge.end(); ++map_gp_itr) {
	            temp_gp2 = map_gp_itr->second->Gp;
	            if(temp_gp2->root->prims_select) continue;
	            if (p->root != temp_gp2->root) {
	                int edgeLength = map_gp_itr->second->distance + p->terminal_dis + temp_gp2->terminal_dis;
	                if(temp_gp2->root->prims_weight > edgeLength) {
	                	temp_gp2->root->prims_weight = edgeLength;
	                	map_gp_itr->second->source = p;
	                	temp_gp2->root->prims_edge = map_gp_itr->second;
	                	FibH.decrease_key(temp_gp2->root->Fnode, edgeLength);
	                }
	            }
	        }
    	}

        p = p->parent;
    }
}
