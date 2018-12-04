#ifndef LOWGRAPH_HPP
#define LOWGRAPH_HPP

#include <vector>
#include <iostream>

using namespace std;

class LowGraph
{
	public:

		LowGraph(double* map, int collision_thresh, int m_Xsz, int m_Ysz); 
		~LowGraph(); 

		struct LowVertex
		{
			LowVertex* m_parent;
			double m_gValue;
			double m_hValue;
			double m_fValue;

			int m_X; int m_Y;  // x and y index 
			int m_Xsz; int m_Ysz; // map size

			LowVertex(pair<int, int> indices, int xsz, int ysz)
			{
					m_X = indices.first;
					m_Y = indices.second;
					m_Xsz = xsz;
					m_Ysz = ysz;
			}

			void SetFValue(double gVal, double hVal)
			{
				m_gValue = gVal;
				m_hValue = hVal;
				m_fValue = gVal + hVal;
			}
		};

		double* m_map;
		int m_CollThresh;
		int m_Xsz;
		int m_Ysz;
		
		vector<LowVertex*> GetSuccessorsLow(LowVertex* vertex); // see MutliSurveillance.cpp
		vector<LowVertex*> LowSearch(pair<int, int>& start, pair<int, int>& goal, int robot); // Make sure to get this pair of start and goal in mid search 
		double LowSearchCost(pair<int, int>& start, pair<int, int>& goal, int robot); // separate function that return only the cost, not the path 
		int LowHash(LowVertex* vertex);
		
};

#endif 