#include <iostream>
#include <unordered_map>
#include <vector>

using namespace std; 

#define NUM_OF_ROBOTS 2
#define NUM_OF_WAYPOINTS 2

const int mxSz = 64; // maximum size of grid 
int dijDist[mxSz][mxSz]; // To store shortest distance between any two nodes 

unordered_map<int, pair<int, int> > waypoint_id_to_grid_location; 

int n; // 8 x 8 grid - 64 nodes - not the poses which lie inside obstacles

// To initialize nodes and edges 
void init(vector<vector<int> >& map)
{
	n = 0; 
	for (int i=0; i < map.size(); i++)
	{
		for (int j=0; j < map[0].size(); j++)
		{
			if (map[i][j]!=1)
				n++;
		}
	}
	
	cout << n << endl;
	return; 
} 

void fw()
{

}

int main()
{
	vector<vector<int> > map{{0,0,1,1,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,1,1,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}}; // initialize a 2D map

	// find a shortest path between any two nodes using floyd-warshall algorithm 
	init(map);

	pair<int, int> start = {0,0}; // 2 Robots 
	pair<int, int> wp1 = {3,7};
	pair<int, int> wp2 = {7,7};

	int M = NUM_OF_WAYPOINTS;
	waypoint_id_to_grid_location[0] = start;
	waypoint_id_to_grid_location[1] = wp1;
	waypoint_id_to_grid_location[2] = wp2;
	waypoint_id_to_grid_location[3] = start; // goal is same as start  

	return 0; 

}
