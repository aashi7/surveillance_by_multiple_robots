#include "GraphVertex.hpp"

#include <vector>
#include <limits>
#include <iostream>
#define D_INF numeric_limits<double>::infinity()

using namespace std;

GraphVertex::GraphVertex(){}

GraphVertex::GraphVertex(vector<int> perWayPtRobots, int lastWayPt, vector<double> perRobotCosts, 
	GraphVertex* parent, vector<GraphVertex*> mid_level_path)
{
	m_WayPtAssignment = perWayPtRobots;
	m_lastAssigned = lastWayPt;
	m_WayPtAssignmentCosts = perRobotCosts;
	m_parent = parent;
	m_mid_level_path = mid_level_path;
}

GraphVertex::GraphVertex(vector<bool> wayPtsDone, int lastWayPt, vector<GraphVertex*> low_level_path)
{
	m_WayPtVisitation = wayPtsDone;
	m_lastVisited = lastWayPt;
	m_low_level_path = low_level_path;
}

GraphVertex::GraphVertex(pair<int, int> indices, int xsz, int ysz)
{
	m_X = indices.first;
	m_Y = indices.second;
	m_Xsz = xsz;
	m_Ysz = ysz;
}

GraphVertex::~GraphVertex(){}

void GraphVertex::SetFValue(double gVal, double hVal)
{
	m_gValue = gVal;
	m_hValue = hVal;
	m_fValue = gVal + hVal;
}

ostream& operator<<(ostream& os, const GraphVertex& vertex)
{
	if ((vertex.m_WayPtAssignment).size() > 0) // indicares it's a top level node 
	{
		os << "\nWay Point Assignment: ";
		for (auto i: vertex.m_WayPtAssignment)
			os << i << " ";
		os << "\nPer Robot Costs: ";
		for (auto i: vertex.m_WayPtAssignmentCosts)
			os << i << " ";
		os << "\nLast Way Point Assigned: " << vertex.m_lastAssigned;
	}

	else if ((vertex.m_WayPtVisitation).size() > 0) // indicates it's a mid level node 
	{
		os << "\nWay Points Visited: ";
		for (auto i: vertex.m_WayPtVisitation)
			os << i << " ";
		os << "\nLast Way Point Visited: " << vertex.m_lastVisited;
	}

	else
	{
		os << "\nGrid coordinates:\n";
		os << "x: " << vertex.m_X << " y: " << vertex.m_Y;
	}

	os << "\nG-Value: " << vertex.m_gValue; 
	return os;
}