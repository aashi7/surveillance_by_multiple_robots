#include "mex.h"
#include <iostream>

#define MAP_IN prhs[0]

#define PLAN_OUT plhs[0]

void planner(double* map, double*** plan)
{
	*plan = NULL; // plan returned 
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
	double* map = mxGetPr(MAP_IN);

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