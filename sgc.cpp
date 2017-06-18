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
	int l_idx=0;
    list < Shape* >::iterator itr1,itr2;
    GraphPoint *gp1, *gp2;
    int x,y;

    //###1. Initialize
    for(int i =0;i<MetalLayers;i++) all_layer[i].SpanningGraphConstruct();

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

    //###3. Construct global graph
    for(size_t s = 0; s < all_line.size(); s++){
    	GP_result = all_layer[all_line[s]->S->layer_position].SGconstruct(all_line[s]);

        if(all_line[s]->S->layer_position>0){
            all_layer[all_line[s]->S->layer_position-1].SGconstruct_search(all_line[s], GP_result.first, GP_result.second);
        }
        if(all_line[s]->S->layer_position<MetalLayers-1){
            all_layer[all_line[s]->S->layer_position+1].SGconstruct_search(all_line[s], GP_result.first, GP_result.second);
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
            gp1->Add_edge(gp2, x, y, x, y,i, 0);
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

    //###4. Undirected Graph (for plot )
    for(int i =0;i<MetalLayers;i++) all_layer[i].ConvertFinalToUndirectedG();



}

void Manager::ExtendedDijkstra(){

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
        if(all_cluster[i]->GetShapeType()==RSHAPE || all_cluster[i]->GetShapeType()==VIA){
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
                if((*gp_itr)->Shape_type==RSHAPE)   sshape = "RSHAPE";
                else if((*gp_itr)->Shape_type==VIA) sshape = "VIA";
                else                                sshape = "OBSTACLE"; 
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

void Manager::addMSTEdges(GraphPoint *p1, GraphPoint *p2) {
    GraphPoint *p = p1;
    int x1,y1,x2,y2;
    MAP_GP_edge::iterator map_gp_itr, map_begin_itr, map_end_itr;
    for(map_gp_itr = p1->map_edge.begin();map_gp_itr!=p1->map_edge.end(); ++map_gp_itr){
        x1 = map_gp_itr->second->point_x1;
        y1 = map_gp_itr->second->point_y1;
        x2 = map_gp_itr->second->point_x2;
        y2 = map_gp_itr->second->point_y2;
        if(p2==map_gp_itr->second->Gp ) {
            //MSTEdges.push_back( Edge(x1, x2, y1, y2) );
            p1->final_edge.push_back(map_gp_itr->second);
        }
    }

    while (p != p->parent) {
        for(map_gp_itr = p->map_edge.begin();map_gp_itr!=p->map_edge.end(); ++map_gp_itr){
            x1 = map_gp_itr->second->point_x1;
            y1 = map_gp_itr->second->point_y1;
            x2 = map_gp_itr->second->point_x2;
            y2 = map_gp_itr->second->point_y2;
            if(p->parent==map_gp_itr->second->Gp ) {
                //MSTEdges.push_back( Edge(x1, x2, y1, y2) );
                p->final_edge.push_back(map_gp_itr->second);
            }
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
            if(p->parent==map_gp_itr->second->Gp ) {
                //MSTEdges.push_back( Edge(x1, x2, y1, y2) );
                 p->final_edge.push_back(map_gp_itr->second);
            }
        }
        p = p->parent;
    }
}

void Manager::ExtendedKruskal() {
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
        if(all_cluster[i]->GetShapeType()==RSHAPE || all_cluster[i]->GetShapeType()==VIA){
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

    // choose MST points
    while (!HeapBE.empty()) {
        curEdge = HeapBE.begin();
        temp_gp1 = curEdge->second->source;
        temp_gp2 = curEdge->second->Gp;
        GraphPoint *set1 = findSet(temp_gp1->root);
        GraphPoint *set2 = findSet(temp_gp2->root);
        if (set1 != set2) {
            unionSet(set1, set2);
            addMSTEdges(temp_gp1, temp_gp2);
        }
        HeapBE.erase(curEdge);
    }

}









string Manager::itos1(int a) {
    string sign = a<0?"-":"";
    string result = a>0?string(1,(a%10+'0')):string(1,((a=-a)%10+'0'));
    (a/=10)>0?result=itos1(a)+result:result;
    return (sign+result);
 }

