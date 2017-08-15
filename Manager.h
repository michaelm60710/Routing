#ifndef Mgr_H
#define Mgr_H

#include <fstream>
#include "layer.h"

using namespace std;



class Manager
{
public:
	Manager(const char* ,const char* );


	void Parsing(const char* );
	Coords* Parsing_coordinate(string, string);
	Coords* Parsing_via(string);

	void SpanningGraphConstruct(); //Step 1
	void SpanningGraphConstruct_2(); //Step 1
	void SpanningTreeConstruct();  //Step 2
	void SpanningTreeConstruct_2();  //Step 2
	void Optimize1(GraphPoint *, GraphPoint *);
	int opt1_shape(Edge_info*, int&, int&, int&, int&);
	void Recur_parent_opt1(GraphPoint* );
	void ExtendedDijkstra();
	void ExtendedDijkstra_2();
	void ExtendedKruskal();
	void ExtendedKruskal_2();
	void ExtendedPrims();
	void update_path(GraphPoint*, FibHeap<int> &);
	void addMSTEdges(GraphPoint *p1, GraphPoint *p2, bool);
	void add_Final_GP(GraphPoint *p1, GraphPoint *p2, bool);
	void unionSet( GraphPoint *, GraphPoint *);
	GraphPoint* findSet(GraphPoint *);

	void Reconstruct();

	void Output(const char* );
	void Output_test(const char* );
	void Verify();

	string itos1(int);//integer convert to string
private:
	int ViaCost;
	int Spacing;
	Coords *Boundary;
	int MetalLayers;
	int RoutedShapes;
	int RoutedVias;
	int Obstacles;
	int MaxRshapeEdge;

	int min_x;
	Cluster *Extra_Obs; //0809 like VIA
	vector < Layer > all_layer;
	
	vector < Shape* > all_shape;//via, obstacle, rshape
	vector < Line*  > all_line;
	vector <Cluster*> all_cluster;
	
	//GP has final edge
	list < GraphPoint* > gp_list;

	//GP sgc2
	vector<GraphPoint* > gp_vec;
	vector<GraphPoint* > all_gp_vec; //gp_vec + rshape + via
	vector<GraphPoint* >::iterator all_gp_rshape_begin;

};


#endif
