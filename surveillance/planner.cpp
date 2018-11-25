#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>

#include "MultiSurveillance.hpp"
#include "ComparePriority.cpp"
#include "HashTop.cpp"
#include "HashMid.cpp"

#define D_INF numeric_limits<double>::infinity()
using namespace std;

int main()
{
	vector<pair<int,int>> starts, goals, wayPts;
	
	starts.push_back({0,0});
	starts.push_back({0,0});
	
	goals.push_back({0,0});
	goals.push_back({0,0});

	wayPts.push_back({3,7});
	wayPts.push_back({7,7});

	int M = wayPts.size(), N = starts.size();

	MultiSurveillance *MS = new MultiSurveillance(M,N,wayPts,starts,goals);
	cout << "Path cost = " << (MS->TopSearch())->m_gValue << '\n';

	return 0; 
}