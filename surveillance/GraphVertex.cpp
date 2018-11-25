#include "GraphVertex.hpp"
using namespace std;

GraphVertex::GraphVertex(){}

GraphVertex::GraphVertex(vector<int> perWayPtRobots, int lastWayPt, 
	vector<double> perRobotCosts, GraphVertex* parent)
{
	m_WayPtAssignment = perWayPtRobots;
	m_lastAssigned = lastWayPt;
	m_WayPtAssignmentCosts = perRobotCosts;
	m_parent = parent;
}

GraphVertex::GraphVertex(vector<bool> wayPtsDone, int lastWayPt)
{
	m_WayPtVisitation = wayPtsDone;
	m_lastVisited = lastWayPt;
}

GraphVertex::~GraphVertex(){}

void GraphVertex::SetFValue(double gVal, double hVal)
{
	m_gValue = gVal;
	m_hValue = hVal;
	m_fValue = gVal + hVal;
}

