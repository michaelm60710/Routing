#ifndef LAYER_H
#define LAYER_H


#include "cluster.h"

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
    GraphPoint* SGconstruct_2(Line*, GraphPoint*, bool); //same layer shape
    void SGconstruct_search(Line*, GraphPoint*, GraphPoint*); //dif layer shape
    void SGcons_RshapeOverlap(Line*, GraphPoint* &, GraphPoint* &, bool &, bool &);
    void SGcons_RshapeOverlap_2(GraphPoint*);
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

	vector<Cluster*> all_cluster;
	list < Shape* > Via_list;
	list < Shape* > Upper_Via_list;
	static int G_point_num;

	//for layout
	vector <GraphPoint*> layer_gp_vec;
	
	//0809
	Cluster *Extra_local_Obs;
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

	


};






#endif
