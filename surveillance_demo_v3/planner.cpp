#include "mex.h"
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>
#include <ctime>

#include "TopGraph.hpp"

using namespace std;

#define MAP_IN prhs[0]
#define COLLISION_THRESH prhs[1]
#define WAYPOINT_IN prhs[2]
#define ROBOTS_START prhs[3]
#define ROBOTS_GOAL prhs[4]

#define PLAN_OUT plhs[0]  // 3-dim double array OUT 

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

void planner(double* map, int collision_thresh, int x_size, int y_size, vector<pair<int,int>>& wayPts,
	vector<pair<int,int>>& starts, vector<pair<int,int>>& goals,
 vector<vector<pair<int,int>>>& plan)
{

	// cout << "Waypoint Read: \n";
	// printCoordinates(wayPts);

	// cout << "Start Positions: \n";
	// printCoordinates(starts);

	// cout << "Goal Positions: \n";
	// printCoordinates(goals); 

	int M = wayPts.size();
	int N = starts.size();

	clock_t begin_time;
	begin_time = clock();
	TopGraph::TopGraph *GT = new TopGraph(M, N, wayPts, starts, goals, map, collision_thresh, x_size, y_size);
	vector<vector<pair<int,int>>> paths_of_all_robots = GT->TopSearch();
	//vector<vector<pair<int,int>>> paths_of_all_robots = GT->GreedyAssignment();
	cout <<  "Planning time: " << float(clock() - begin_time)/CLOCKS_PER_SEC << endl;
	GT->~TopGraph();

	plan = paths_of_all_robots;
	return;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	/* Check for proper number of arguments */
	if (nrhs != 5) // as a first step, only map 
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
	double* map =  mxGetPr(MAP_IN);

	int numofDOFs = 2;

	int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
	// Read robot start positions, goal positions and waypoint position from MATLAB too 
	
	////////// Waypoints ///////////////////////
	int numWayPts = (int) mxGetN(WAYPOINT_IN);
	vector<pair<int,int>> wayPts(numWayPts); 
	double* waypoints = mxGetPr(WAYPOINT_IN);

	for (int i = 0; i < numWayPts; i++)
	{
		wayPts[i].first = (int) waypoints[2*i]; // from MATLAB indexing to C++ indexing 
		wayPts[i].second = (int) waypoints[2*i+1]; 
	}

	////////// Robot Starts ////////////////
	int numRobots = (int) mxGetN(ROBOTS_START);
	//cout << numRobots << endl;

	vector<pair<int,int>> starts(numRobots);
	double* robotStarts = mxGetPr(ROBOTS_START);

	for (int i = 0; i < numRobots; i++)
	{
		starts[i].first = (int) robotStarts[2*i]; // from MATLAB indexing to C++ indexing 
		starts[i].second = (int) robotStarts[2*i+1];
	}
	
	/////////// Robot Goals /////////////
	vector<pair<int,int>> goals(numRobots);
	double* robotGoals = mxGetPr(ROBOTS_GOAL);

	for (int i = 0; i < numRobots; i++)
	{
		goals[i].first = (int) robotGoals[2*i]; // from MATLAB indexing to C++ indexing 
		goals[i].second = (int) robotGoals[2*i+1];
	}

	/* Returning the plan */
	//double** plan = NULL;
	vector<vector<pair<int,int>>> plan; 
	// now call the function here 
	planner(map, collision_thresh, x_size, y_size, wayPts, starts, goals, plan);

	int max_plan_length = 0;
	for (int r = 0; r < plan.size(); r++)
	{
		if (max_plan_length < plan[r].size())
			max_plan_length  = plan[r].size();
	}

	int ndim = 3;
	int N = plan.size();
	double *plan_out;

	if (plan.size() == 0)
	{
		const mwSize dims[3] = {(mwSize)numRobots, (mwSize)max_plan_length, (mwSize)2};
		PLAN_OUT = mxCreateNumericArray((mwSize)ndim, dims, mxDOUBLE_CLASS, mxREAL);	
		plan_out = mxGetPr(PLAN_OUT);
		return;
	}

	const mwSize dims[3] = {(mwSize)N, (mwSize)max_plan_length, (mwSize)2};

	PLAN_OUT = mxCreateNumericArray((mwSize)ndim, dims, mxDOUBLE_CLASS, mxREAL);
	
	plan_out = mxGetPr(PLAN_OUT);


	for(int i = 0; i < numRobots; i++)
    {
        for(int j = 0; j < plan[i].size(); j++)
//            for (int k = 0; k < 2; k++)
            {
                plan_out[0*max_plan_length*numRobots + j*numRobots + i] = plan[i][j].first;
                plan_out[1*max_plan_length*numRobots + j*numRobots + i] = plan[i][j].second;
            }
        int end = plan[i].size() - 1;
        for(int j = plan[i].size(); j < max_plan_length; j++)
            //for (int k = 0; k < 2; k++)
        {

            plan_out[0*max_plan_length*numRobots + j*numRobots + i] = plan[i][end].first;
            plan_out[1*max_plan_length*numRobots + j*numRobots + i] = plan[i][end].second;
        }
    }

	return;
}