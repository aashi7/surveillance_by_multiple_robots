#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>
#include <math.h>
#include "mex.h"

#include "MultiSurveillance.hpp"
#include "GraphVertex.hpp"
#include "ComparePriority.cpp"

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	COLLISION_THRESH        prhs[1]
#define	ROBSTARTS_IN			prhs[2]
#define	ROBGOALS_IN             prhs[3]
#define	WAYPTS_IN				prhs[4]


/* Output Arguments */
#define PLANS_OUT                plhs[0]
#define	PLANLENGTHS_OUT          plhs[1]

#define D_INF numeric_limits<double>::infinity()
using namespace std;

static void planner(
    double* map,
    int collision_thresh,
    int x_size,
    int y_size,
    vector<pair<int,int>> starts,
    vector<pair<int,int>> goals,
    vector<pair<int,int>> wayPts,
    int numRobots,
    int numWayPts,
    double**** plansPtr,
    int** planLengthsPtr
    )
{
	MultiSurveillance *MS = new MultiSurveillance(numWayPts, numRobots, wayPts, starts, goals, 
                                                        map, collision_thresh, x_size, y_size);
	double*** finalPlans; int* finalPlanLengths;
    tie(finalPlans, finalPlanLengths) = MS->RunPlan(false); //false for optimal search, true for random top-level assignment
    cout << "Planning time = " << MS->m_plantime << " s\n";
    cout << "Total path cost = " << MS->m_plancost << '\n';
    *plansPtr = finalPlans;
    *planLengthsPtr = finalPlanLengths;
}


//prhs contains input parameters (5): 
//1st is matrix with all the obstacles
//2nd is an integer C, the collision threshold for the map
//3rd is matrix of robot starts (2xnumRobots)
//4th is matrix of robot goals (2xnumRobots)
//5th is matrix of waypoint locations (2xnumWayPts)
//plhs should contain output parameters (2): 
//1st is a 3D matrix plans when each plan[i][j] is the 2D position of robot i at jth step of the plan
//2nd is array of planlengths (int*)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 5) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Five input arguments required."); 
    } else if (nlhs != 2) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "Two output arguments required."); 
    } 
        
    /* Get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);

    /* Get number of robots and robot start & goal positions */
    int numRobots = (int) mxGetN(ROBSTARTS_IN);
    double* rob_starts = mxGetPr(ROBSTARTS_IN);
    double* rob_goals = mxGetPr(ROBGOALS_IN);

    vector<pair<int,int>> starts(numRobots);
    vector<pair<int,int>> goals(numRobots);
    for(int i = 0; i < numRobots; i++)
    {
        starts[i].first = (int) rob_starts[2*i];
        starts[i].second = (int) rob_starts[2*i+1];
        goals[i].first = (int) rob_goals[2*i];
        goals[i].second = (int) rob_goals[2*i+1];
    }

    /* Get number of waypoints and waypoint locations */
    int numWayPts = (int) mxGetN(WAYPTS_IN);
    double* waypt_pos = mxGetPr(WAYPTS_IN);

    vector<pair<int,int>> wayPts(numWayPts);
    for(int i = 0; i < numWayPts; i++)
    {
        wayPts[i].first = (int) waypt_pos[2*i];
        wayPts[i].second = (int) waypt_pos[2*i+1];
    }

    /* Do the actual planning */
    double*** plans = NULL;
    int* planlengths = NULL;
    planner(map, collision_thresh, x_size, y_size, starts, goals, wayPts, numRobots, numWayPts, &plans, &planlengths);

    /* Create return values */
    int max_planlength = 0;
    for(int i = 0; i < numRobots; i++)
        if(planlengths[i] > max_planlength)
            max_planlength = planlengths[i];

    const mwSize out_dims[3] = {(mwSize)numRobots, (mwSize)max_planlength, (mwSize)2};

    PLANS_OUT = mxCreateNumericArray( (mwSize)3, out_dims, mxDOUBLE_CLASS, mxREAL); 
    double* plans_out = mxGetPr(PLANS_OUT);        

    for(int i = 0; i < numRobots; i++)
    {
        for(int j = 0; j < planlengths[i]; j++)
            for (int k = 0; k < 2; k++)
                plans_out[k*max_planlength*numRobots + j*numRobots + i] = plans[i][j][k];
        for(int j = planlengths[i]; j < max_planlength; j++)
            for (int k = 0; k < 2; k++)
                plans_out[k*max_planlength*numRobots + j*numRobots + i] = plans[i][planlengths[i]-1][k];
    }


    PLANLENGTHS_OUT = mxCreateNumericMatrix( (mwSize)numRobots, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlengths_out = (int*) mxGetPr(PLANLENGTHS_OUT);
    for(int i = 0; i < numRobots; i++)
        planlengths_out[i] = planlengths[i];

    return;    
}