#ifndef MIDGRAPH_HPP
#define MIDGRAPH_HPP

#include "LowGraph.hpp"

#include <vector>
#include <iostream>

using namespace std;

class MidGraph
{
	public:

		int m_numWayPts; 

		vector<pair<int,int>> m_starts;
		vector<pair<int,int>> m_goals;
		vector<pair<int,int>> m_wayPts; 
		//vector<vector<int>> m_map;
		double* m_map;
		int m_CollThresh; 
		int m_Xsz;
		int m_Ysz;

		MidGraph(int numWayPts, vector<pair<int,int>> starts, vector<pair<int,int>> goals, vector<pair<int,int>> wayPts, 
			double* map, int collision_thresh, 
			int x_size, int y_size); 
		~MidGraph(); 

		struct MidVertex
		{
			vector<bool> m_WayPtVisitation;
			int m_lastVisited;
			vector<LowGraph::LowVertex*> m_low_level_path;
			double m_gValue;
			double m_hValue;
			double m_fValue;
			MidVertex* m_parent;

			MidVertex(vector<bool> wayPtsDone, int lastWayPt, vector<LowGraph::LowVertex*> m_low_level_path)
			{
				m_WayPtVisitation = wayPtsDone;
				m_lastVisited = lastWayPt;
				m_low_level_path = m_low_level_path;
			}

			void SetFValue(double gVal, double hVal)
			{
				m_gValue = gVal;
				m_hValue = hVal;
				m_fValue = gVal + hVal;
			}
		};
		
		vector<MidVertex*> GetSuccessorsMid(MidVertex* vertex, MidVertex* end); // see MutliSurveillance.cpp
		vector<MidVertex*> MidSearch(vector<int>& WayPtAssignment, int robot);
		double MidSearchCost(vector<int>& WayPtAssignment, int robot);
		int MidHash(MidVertex* vertex);
		MidVertex* GetMidVertex(vector<int>& WayPtAssignment, int robot);
};

#endif 