#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>

#include "MultiSurveillance.hpp"
#include "ComparePriority.cpp"

#define D_INF numeric_limits<double>::infinity()
using namespace std;

int main()
{
	double* map;
	map = (double*) malloc(64*sizeof(double));
	for(int i = 0; i < 64; i++){ map[i] = 1; }
	vector<pair<int,int>> starts, goals, wayPts;
	
	starts.push_back({1,1});
	starts.push_back({8,8});

	goals.push_back({8,8});
	goals.push_back({1,1});

	wayPts.push_back({4,8});
	wayPts.push_back({8,4});

	int M = wayPts.size(), N = starts.size();

	MultiSurveillance *MS = new MultiSurveillance(M,N,wayPts,starts,goals,map,(double)2,8,8);
	cout << "Path cost = " << (MS->TopSearch())->m_gValue << '\n';
	double ***plans; int *planlengths;
	tie(plans,planlengths) = MS->RunPlan();
	for(int j = 0; j < planlengths[0]; j++)
		cout << plans[0][j][0] << ' ' << plans[0][j][1] << '\n';

	return 0; 
}