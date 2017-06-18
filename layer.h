#ifndef LAYER_H
#define LAYER_H


#include "cluster.h"

using namespace std;



class Layer{
public:
	Layer();
	void SpanningGraphConstruct();            //step 1
	void SpanningTreeConstruct();             //step 2
	void RectilinearSpanningTreeConstruct();  //step 3
	void OARSMT();                            //step 4

	//void MergeCluster(Cluster *, Cluster *);



	void clustering_shape();
	void find_overlap(Shape*, int);
	void BFS_overlap_graph(Shape *, Cluster*);
    void build_clu(Shape*, Cluster*, int);
    bool overlap(Shape*, Shape*);

    //
    void SGconstruct();
    pair<GraphPoint*, GraphPoint*> SGconstruct(Line*); //same layer shape
    void SGconstruct_search(Line*, GraphPoint*, GraphPoint*); //dif layer shape
    void SG_find_GPinfo(int , BoundLine_info*);
    void ConvertToUndirectedG();
    void ExtendedDijkstra();
    void ExtendedKruskal();
    void check_point_svg();

	//for private use
	void Rshape_list_append(Shape *);
	void Obstacle_list_append(Shape *);
	void Via_list_append(Shape *);
	int get_Rshape_num();
	int get_Obstacle_num();
	int get_Via_num();

	//Extended Kruskal's
	void addMSTEdges(GraphPoint *p1, GraphPoint *p2);
	int Spacing;
	int Width;
	int Height;
	int Layer_pos;
	int Via_cost;

private:
	list < Shape* > Rshape_list;
	list < Shape* > Obstacle_list;
	list < Shape* > Via_list;
	vector<Cluster*> all_cluster;
	vector <Shape* > all_shape_vec;
    vector <GraphPoint* > all_GP_vec;
	vector<Shape*> sort_by_x1;
	vector<Shape*> sort_by_y1;
	//vector <Shape* > X_sort_shape;
	//MAP_Shape X_msort_shape; //sort X2
	//MAP_Shape Y_msort_shape; //sort Y2
	int Rshape_num;
	int Obstacle_num;
	int Via_num;
	int Layer_Shape_num;
	int G_point_num;
	//step 1

	//Extended Kruskal's
	list <Edge> MSTEdges;

	map< int , BoundLine_info* , less<int> > bound_map;

};






#endif
