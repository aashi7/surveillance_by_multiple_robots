#ifndef TOPGRAPH_HPP
#define TOPGRAPH_HPP

#include <vector>
#include <iostream>
#include <unordered_map>

#include "MidGraph.hpp"

using namespace std;

class TopGraph
{
	public:

		int m_numWayPts; 
		int m_numRobots;
		vector<pair<int,int>> m_starts;
		vector<pair<int,int>> m_goals;
		vector<pair<int,int>> m_wayPts;
		//vector<vector<int>> m_map;
		double* m_map;
		int m_Xsz;
		int m_Ysz;

		int m_CollThresh;

		TopGraph(int numWayPts, int numRobots, vector<pair<int,int>> wayPts, vector<pair<int,int>> starts, vector<pair<int,int>> goals, 
			double* map, int collision_thresh, 
			int x_size, int y_size); 
		~TopGraph(); 

		struct TopVertex
		{
			vector<int> m_WayPtAssignment;
			int m_lastAssigned;
			vector<double> m_WayPtAssignmentCosts;
			vector<MidGraph::MidVertex*> m_mid_level_path;
			double m_gValue;
			double m_hValue;
			double m_fValue;
			TopVertex* m_parent; 

			TopVertex(vector<int> perWayPtRobots, int lastWayPt, 
			vector<double> perRobotCosts, TopVertex* parent, vector<MidGraph::MidVertex*> mid_level_path)
			{
					m_WayPtAssignment = perWayPtRobots;
					m_lastAssigned = lastWayPt;
					m_WayPtAssignmentCosts = perRobotCosts;
					m_parent = parent;
					m_mid_level_path = mid_level_path;
			}

			void SetFValue(double gVal, double hVal)
			{
				m_gValue = gVal;
				m_hValue = hVal;
				m_fValue = gVal + hVal;
			}
		};
		
		vector<TopVertex*> GetSuccessorsTop(TopVertex* vertex, unordered_map<int, TopVertex*> vertices); 
		int TopHash(TopVertex* vertex);
		double TopPathCost(TopVertex* current, TopVertex* successor); 
		vector<vector<pair<int, int>>> TopSearch();
};

#endif 