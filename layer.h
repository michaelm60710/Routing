#ifndef LAYER_H
#define LAYER_H


#include "cluster.h"
class Edge_info2;

using namespace std;


class Layer{
public:
	Layer();
	void SpanningGraphConstruct();            //step 1
	void SpanningGraphConstruct_2();            //step 1.2
	//void SpanningTreeConstruct();             //step 2

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

    //0812
    void Extra_obs(Line*, GraphPoint* &, GraphPoint* &, int, int, int, int &, int &);
    void Extra_obs_RSHAPE_right(Line*, GraphPoint* &, GraphPoint* &, int, int, int, int &, int &, map< int , BoundLine_info* , less<int> > &);
    GraphPoint* SGconstruct_extra_obs(const int, const int, const int, GraphPoint*, int, int &);
    GraphPoint* SGconstruct_extra_obs_RSHAPE_right(const int, const int, const int, GraphPoint*, int, int &, map< int , BoundLine_info* , less<int> > &);
    void Update_Rbound_map(const int, const int, const int, map< int , BoundLine_info* , less<int> > &);
    int Get_better_x_pos(int, int, int, int, map< int , BoundLine_info* , less<int> >::iterator &);//0816
    int Get_better_x_pos(int, int, int);//0826
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
	int Max_dis;
	int Max_ob_dis;//1116

	//dff_map
	void diff_map_update(Line*); //same layer shape
	void diff_map_insert_rshape_point(GraphPoint*, const int, const int);

	vector<Cluster*> all_cluster;
	list < Shape* > Via_list;
	list < Shape* > Upper_Via_list;
	static int G_point_num;

	//for layout
	vector <GraphPoint*> layer_gp_vec;
	
	//0809
	Cluster *Extra_local_Obs;
	//1117
	int g_obs_len;
	vector <GraphPoint*> Obs_gp_vec;

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

	map< int , BoundLine_info* , less<int> > bound_map;
	map< int , BoundLine_info* , less<int> > dff_bound_map; //dfferent Rshape 0827

	// 1102 store the shortest edge in each region for each Gp
	Edge_info2 mmin_edge, mmin_edge2;


};





#endif
