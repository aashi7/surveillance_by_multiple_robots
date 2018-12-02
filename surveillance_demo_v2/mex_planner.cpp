#include "mex.h"
#include "matrix.h" // mwSize, mwIndex 
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>

#include "MultiSurveillance.hpp"
#include "ComparePriority.cpp"

#define D_INF numeric_limits<double>::infinity()
using namespace std;

#define MAP_IN prhs[0]
#define WAYPOINT_IN prhs[1]
#define ROBOTS_START prhs[2]
#define ROBOTS_GOAL prhs[3]
#define PLAN_OUT plhs[0]  // 3-dim double array OUT 

using namespace std;

///// Print a vector<pair<int,int>> ////////
void printCoordinates(vector<pair<int,int>>& arr)
{
	int n = arr.size();
	for (int i = 0; i < n; i++)
	{
		cout << arr[i].first << " " << arr[i].second << endl;
	}
	return; 
}

//void planner(vector<vector<int>>& map, double*** plan)
void planner(vector<vector<int>>& map, vector<pair<int,int>>& wayPts,
	vector<pair<int,int>>& starts, vector<pair<int,int>>& goals,
 vector<vector<pair<int,int>>>& plan)
{

	// vector<pair<int,int>> starts, goals, wayPts;

	// starts.push_back({0,0});
	// starts.push_back({0,0});

	// goals.push_back({0,0});
	// goals.push_back({0,0});

	// wayPts.push_back({3,7});
	// wayPts.push_back({7,7});

	cout << "Waypoint Read: \n";
	printCoordinates(wayPts);

	cout << "Start Positions: \n";
	printCoordinates(starts);

	cout << "Goal Positions: \n";
	printCoordinates(goals); 

	int M = wayPts.size();
	int N = starts.size();

	int xsz = map[0].size();
	int ysz = map.size();

	// int map[8][8] = 
	// {
	// 	{0,0,0,0,0,0,0,0},
	// 	{0,0,0,0,0,0,0,0},
	// 	{0,0,0,0,0,0,0,0},
	// 	{0,0,0,0,0,0,0,0},
	// 	{0,0,0,0,0,0,0,0},
	// 	{0,0,0,0,0,0,0,0},
	// 	{0,0,0,0,0,0,0,0},
	// 	{0,0,0,0,0,0,0,0}
	// };

	//vector<vector<int>> map(8, vector<int>(8,0)); // 8 x 8 with zeros 

	MultiSurveillance *MS = new MultiSurveillance(M, N, wayPts, starts, goals, map, xsz, ysz);
	// cout << "search done" << endl;
	// cout << (MS->TopSearch())->m_gValue << " Path cost" << endl;

	vector<vector<pair<int,int>>> paths_of_all_robots = MS->TopSearch();

	    for (int r = 0; r < N; r++)
        {
            cout << r << endl;
            for (int j = 0; j < paths_of_all_robots[r].size(); j++)
            {
                cout << paths_of_all_robots[r][j].first << " " << paths_of_all_robots[r][j].second << endl;
            }
        }

    // To return the path in MATLAB array 
	//*plan = paths_of_all_robots[0]; // returning the path of one robot 
	plan = paths_of_all_robots;
	return;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	/* Check for proper number of arguments */
	if (nrhs != 4) // as a first step, only map 
	{
		mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs", "Map is required.");
	}
	else if (nlhs != 1)
	{
		mexErrMsgIdAndTxt("MATLAB:planner:maxlhs", "Output argument is required.");
	}

	/* get the dimensions of the map and the map matrix itself */
	int x_size = (int) mxGetM(MAP_IN); // M x N matrix map 
	int y_size = (int) mxGetN(MAP_IN);
	double* input_map = mxGetPr(MAP_IN);

	vector<vector<int>> map(y_size, vector<int>(x_size,0));
	int ptr = 0;
	for (int i = 0; i < y_size; i++)
	{
		for (int j = 0; j < x_size; j++)
		{
			map[i][j] = input_map[ptr];
			ptr++;
		}
	}

	int numofDOFs = 2;

	// Read robot start positions, goal positions and waypoint position from MATLAB too 
	
	////////// Waypoints ///////////////////////
	int numWayPts = (int) mxGetN(WAYPOINT_IN);
	vector<pair<int,int>> wayPts(numWayPts); 
	double* waypoints = mxGetPr(WAYPOINT_IN);

	ptr = 0;
	for (int i = 0; i < numWayPts; i++)
	{
		wayPts[i].first = waypoints[ptr] - 1; // from MATLAB indexing to C++ indexing 
		ptr++;
		wayPts[i].second = waypoints[ptr] - 1; 
		ptr++;
	}

	////////// Robot Starts ////////////////
	int numRobots = (int) mxGetN(ROBOTS_START);
	cout << numRobots << endl;
	vector<pair<int,int>> starts(numRobots);
	double* robotStarts = mxGetPr(ROBOTS_START);

	ptr = 0;
	for (int i = 0; i < numRobots; i++)
	{
		starts[i].first = robotStarts[ptr] - 1; // from MATLAB indexing to C++ indexing 
		ptr++;
		starts[i].second = robotStarts[ptr] - 1;
		ptr++;
	}
	
	/////////// Robot Goals /////////////
	vector<pair<int,int>> goals(numRobots);
	double* robotGoals = mxGetPr(ROBOTS_GOAL);

	ptr = 0;
	for (int i = 0; i < numRobots; i++)
	{
		goals[i].first = robotGoals[ptr] - 1; // from MATLAB indexing to C++ indexing 
		ptr++;
		goals[i].second = robotGoals[ptr] - 1;
		ptr++;
	}

	//double** plan = NULL;
	vector<vector<pair<int,int>>> plan; 
	// now call the function here 
	planner(map, wayPts, starts, goals, plan);

	int max_plan_length = 0;
	for (int r = 0; r < plan.size(); r++)
	{
		if (max_plan_length < plan[r].size())
			max_plan_length  = plan[r].size();
	}

	int ndim = 3;
	int N = plan.size();

	const mwSize dims[3] = {(mwSize)numofDOFs, (mwSize)max_plan_length, (mwSize)N};

	PLAN_OUT = mxCreateNumericArray((mwSize)ndim, dims, mxDOUBLE_CLASS, mxREAL);
	double *plan_out;
	plan_out = mxGetPr(PLAN_OUT);
	// for (int r = 0; r < plan.size(); r++)
	// {
	// 	int planlength = plan[r].size(); // To assign dimensions to output matrix 
	// 	int numofDOFs = 2; // in x and in y 
	// 	if (planlength > 0)
	// 	{
	// 		//PLAN_OUT = mxCreateNumericMatrix((mwSize)planlength, (mwSize)numofDOFs, mxINT16_CLASS, mxREAL);
	// 		PLAN_OUT = mxCreateNumericMatrix((mwSize)ndim, (mwSize)dims, mxDOUBLE_CLASS, mxREAL);
	// 		double* plan_out = mxGetPr(PLAN_OUT);
	// 		// copy the values 
	// 		int i,j;
	// 		for (i = 0; i < max_plan_length; i++)
	// 		{
	// 				plan_out[r*max_plan_length*2 + i*2 + 0] = plan[r][i].first;
	// 				plan_out[r*max_plan_length*2 + i*2 + 1] = plan[r][i].second; // Memory error 
	// 		}
	// 	}
	// 	else
	// 	{
	// 		PLAN_OUT = mxCreateNumericMatrix((mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL);
	// 		double* plan_out = mxGetPr(PLAN_OUT);
	// 		int j;
	// 		for (j = 0; j < numofDOFs; j++)
	// 		{
	// 			plan_out[j] = 0;
	// 		}
	// 	}

	// }

	int dim3, dim2;
	for (dim3 = 0; dim3 < dims[2]; dim3++)
	{
		for (dim2 = 0; dim2 < dims[1]; dim2++)
		{
			if (dim2 < plan[dim3].size())
			{
				plan_out[(dim3) + (dim2)*(dims[2]) + 0*dims[2]*dims[1]] = plan[dim3][dim2].first;
				plan_out[(dim3) + (dim2)*(dims[2]) + 1*dims[2]*dims[1]] = plan[dim3][dim2].second;
			}
		}
	}

	return;
}


