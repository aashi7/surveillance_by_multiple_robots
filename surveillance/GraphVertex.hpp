#ifndef GRAPHVERTEX_HPP
#define GRAPHVERTEX_HPP

#include <vector>
#include <limits>
#define D_INF numeric_limits<double>::infinity()

using namespace std;

/* Class for vertex in graphs */
class GraphVertex
{
public:
	GraphVertex();
	GraphVertex(vector<int> perWayPtRobots, int lastWayPt, 
				vector<double> perRobotCosts, GraphVertex* parent);
	GraphVertex(vector<bool> wayPtsDone, int lastWayPt);
	~GraphVertex();
	
	void SetFValue(double gVal, double hVal);

	
	// Vertex for top level graph - search top graph for final answer
	vector<int> m_WayPtAssignment;         /*  M vector:   each waypoint is assigned to a robot    */
	int m_lastAssigned;	                   /*  int id:     last waypoint to be assigned            */
	vector<double> m_WayPtAssignmentCosts; /*  N vector:   cost of assigning waypts to each robot  */
	


	// Vertex for mid level graph - one mid graph per robot
	vector<bool> m_WayPtVisitation;        /*  M vector:   which waypoints have been visited?      */
	int m_lastVisited;                     /*  int id:     which waypoint was visited last?        */


	GraphVertex* m_parent;                 /*  vertex parent   */
	double m_gValue;                       /*  cost from start */
	double m_hValue;                       /*  heuristic value */
	double m_fValue;                       /*  priority value  */
	

};

#endif