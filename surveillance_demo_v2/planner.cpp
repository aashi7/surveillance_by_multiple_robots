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
	cout << (MS->TopSearch()).back()->m_gValue << " Path cost" << endl;


	return 0;
}