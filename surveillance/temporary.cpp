#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>

#include "mex.h"

#include "MultiSurveillance.hpp"
#include "ComparePriority.cpp"

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	COLLISION_THRESH        prhs[1]
#define	ROBSTARTS_IN			prhs[2]
#define	ROBGOALS_IN             prhs[3]
#define	WAYPTS_IN				prhs[4]


/* Output Arguments */
#define PLAN_OUT                plhs[0]
#define	PLANLENGTH_OUT          plhs[1]

#define D_INF numeric_limits<double>::infinity()
using namespace std;

int main()
{
	vector<pair<int,int>> starts, goals, wayPts;
	
	starts.push_back({0,0});
	starts.push_back({10,10});
	starts.push_back({0,10});
	starts.push_back({10,0});

	goals.push_back({10,10});
	goals.push_back({0,0});
	goals.push_back({10,0});
	goals.push_back({0,10});

	wayPts.push_back({3,7});
	wayPts.push_back({7,7});
	wayPts.push_back({4,2});
	wayPts.push_back({6,8});
	wayPts.push_back({1,1});

	int M = wayPts.size(), N = starts.size();

	MultiSurveillance *MS = new MultiSurveillance(M,N,wayPts,starts,goals,NULL,0,0,0);
	cout << "Path cost = " << (MS->TopSearch())->m_gValue << '\n';

	return 0; 
}


//prhs contains input parameters (5): 
//1st is matrix with all the obstacles
//2nd is an integer C, the collision threshold for the map
//3rd is matrix of robot starts (2xnumRobots)
//4th is matrix of robot goals (2xnumRobots)
//5th is matrix of waypoint locations (2xnumWayPts)
//plhs should contain output parameters (2): 
//1st is a 3D matrix plan when each plan[i][j] is the 2D position of robot i at jth step of the plan
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 2) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Two input arguments required."); 
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
    int* rob_starts = mxGetPr(ROBSTARTS_IN);
    int* rob_goals = mxGetPr(ROBGOALS_IN);

    /* Get number of waypoints and waypoint locations */
    int numWayPts = (int) mxGetN(WAYPTS_IN);
    int* waypt_pos = mxGetPr(WAYPTS_IN);

    //call the planner
    double** plan = NULL;
    int planlength = 0;

    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, starts, goals, wayPts, numRobots, numWayPts, &plan, &planlength);

    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < 2; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    
    return;
    
}