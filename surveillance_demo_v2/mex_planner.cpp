#include "mex.h"
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
#define PLAN_OUT plhs[0]

using namespace std;

void planner(vector<vector<int>>& input_map, double*** plan)
{

	vector<pair<int,int>> starts, goals, wayPts;

	starts.push_back({0,0});
	starts.push_back({0,0});

	goals.push_back({0,0});
	goals.push_back({0,0});

	wayPts.push_back({3,7});
	wayPts.push_back({7,7});

	int M = wayPts.size();
	int N = starts.size();

	int xsz = 8;
	int ysz = 8;

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

	vector<vector<int>> map(8, vector<int>(8,0)); // 8 x 8 with zeros 

	MultiSurveillance *MS = new MultiSurveillance(M, N, wayPts, starts, goals, map, xsz, ysz);
	// cout << "search done" << endl;
	cout << (MS->TopSearch())->m_gValue << " Path cost" << endl;
	*plan = NULL;
	return;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	/* Check for proper number of arguments */
	if (nrhs != 1) // as a first step, only map 
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

	int planlength = 0; // To assign dimensions to output matrix 
	double** plan = NULL;
	// now call the function here 
	planner(map, &plan);

	int numofDOFs = 2; // in x and in y 
	if (planlength > 0)
	{
		PLAN_OUT = mxCreateNumericMatrix((mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL);
		double* plan_out = mxGetPr(PLAN_OUT);
		// copy the values 
		int i,j;
		for (i = 0; i < planlength; i++)
		{
			for (j = 0; j < numofDOFs; j++)
			{
				plan_out[j*planlength + i] = plan[i][j];
			}
		}
	}
	else
	{
		PLAN_OUT = mxCreateNumericMatrix((mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL);
		double* plan_out = mxGetPr(PLAN_OUT);
		int j;
		for (j = 0; j < numofDOFs; j++)
		{
			plan_out[j] = 0;
		}
	}

	return;
}

