#ifndef GRAPHVERTEX_HPP
#define GRAPHVERTEX_HPP

#include <vector>
#include <iostream>

using namespace std;

/* Class for vertex in graphs */
class GraphVertex
{
	public:
		GraphVertex();
		GraphVertex(vector<int> perWayPtRobots, int lastWayPt, 
			vector<double> perRobotCosts, GraphVertex* parent, vector<GraphVertex*> mid_level_path);
		GraphVertex(vector<bool> wayPtsDone, int lastWayPt, vector<GraphVertex*> low_level_path);  
		GraphVertex(pair<int, int> indices, int xsz, int ysz);
		~GraphVertex();

		void SetFValue(double gVal, double hVal);
		friend ostream& operator<<(ostream& os, const GraphVertex& vertex);

		// Vertex for top level graph 
		vector<int> m_WayPtAssignment; // q
		int m_lastAssigned;            // indirect reference to robot_id 
		vector<double> m_WayPtAssignmentCosts;
		vector<GraphVertex*> m_mid_level_path; // Middle level path between two top level nodes 

		// Vertex for mid level graph 
		vector<bool> m_WayPtVisitation; // alpha
		int m_lastVisited; // omega 
		vector<GraphVertex*> m_low_level_path;// Low level path between two mid level nodes 

		// Vertex for low level graph 
		int m_X; int m_Y;  // x and y index 
		int m_Xsz; int m_Ysz; // map size 

		GraphVertex* m_parent;
		double m_gValue;
		double m_hValue;
		double m_fValue;

};

#endif