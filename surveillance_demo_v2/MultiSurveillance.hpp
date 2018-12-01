#ifndef MULTISURVEILLANCE_HPP
#define MULTISURVEILLANCE_HPP

#include "GraphVertex.hpp"
#include <vector>

class MultiSurveillance
{
	public:
		MultiSurveillance(int M, int N, vector<pair<int, int>> wayPts, 
			vector<pair<int,int>> starts, vector<pair<int,int>> goals,
			vector<vector<int>>& map, int xsz, int ysz);
		~MultiSurveillance();

		vector<GraphVertex*> GetSuccessorsTop(GraphVertex* vertex, 
			unordered_map<int, GraphVertex*> vertices); // didnt exactly the purpose of vertices

		double TopPathCost(GraphVertex* current, GraphVertex* successor);
		int TopHash(GraphVertex* vertex);
		GraphVertex* TopSearch();

		vector<GraphVertex*> GetSuccessorsMid(GraphVertex* vertex, GraphVertex* end);
		GraphVertex* GetMidVertex(GraphVertex* goal, int robot);
		int MidHash(GraphVertex* vertex);
		double MidSearch(GraphVertex* goal, int robot);

		vector<GraphVertex*> GetSuccessorsLow(GraphVertex* vertex);
		int LowHash(GraphVertex* vertex);
		double LowSearch(GraphVertex* start, GraphVertex* goal, int robot);

		int m_numWayPts;
		int m_numRobots;
		vector<pair<int,int>> m_wayPts;
		vector<pair<int,int>> m_starts;
		vector<pair<int,int>> m_goals;

		int m_mapXsize;
		int m_mapYsize;
		vector<vector<int>> m_map;
};

#endif