#ifndef LAYER_H
#define LAYER_H


#include "cluster.h"

using namespace std;



class Layer{
public:
	Layer();
	void SpanningGraphConstruct();            //step 1
	//void SpanningTreeConstruct();             //step 2
	void RectilinearSpanningTreeConstruct();  //step 3
	void OARSMT();                            //step 4

	//void MergeCluster(Cluster *, Cluster *);

	void clustering_shape();
	void find_overlap(Shape*, int);
	void BFS_overlap_graph(Shape *, Cluster*);
    void build_clu(Shape*, Cluster*, int);
    bool overlap(Shape*, Shape*);

    //
    pair<GraphPoint*, GraphPoint*> SGconstruct(Line*); //same layer shape
    void SGconstruct_search(Line*, GraphPoint*, GraphPoint*); //dif layer shape
    void SGcons_RshapeOverlap(Line*, GraphPoint* &, GraphPoint* &, bool &, bool &);
    void diff_layer_via(Line*, GraphPoint*, GraphPoint*, GraphPoint*, GraphPoint*);
    void ConvertToUndirectedG();
    void check_point_svg(string name="x");

	//for private use
	void Rshape_list_append(Shape *);
	void Obstacle_list_append(Shape *);
	void Via_list_append(Shape *);
	void Upper_Via_list_append(Shape *);
	int get_Rshape_num();
	int get_Obstacle_num();
	int get_Via_num();

	//Extended Kruskal's
	void addMSTEdges(GraphPoint *p1, GraphPoint *p2);
	int Spacing;
	int min_x;
	int Width;
	int Height;
	int Layer_pos;
	int Via_cost;
	vector<Cluster*> all_cluster;
	list < Shape* > Via_list;
	list < Shape* > Upper_Via_list;

private:
	list < Shape* > Rshape_list;
	list < Shape* > Obstacle_list;
	vector <Shape* > all_shape_vec;
    vector <GraphPoint* > all_GP_vec;
	vector<Shape*> sort_by_x1;
	vector<Shape*> sort_by_y1;
	int Rshape_num;
	int Obstacle_num;
	int Via_num;
	int Upper_Via_num;
	int Layer_Shape_num;
	static int G_point_num;

	map< int , BoundLine_info* , less<int> > bound_map;

	


};






#endif
