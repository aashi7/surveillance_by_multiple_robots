#include "GraphVertex.hpp"

#include <vector>
#include <limits>
#include <iostream>
#define D_INF numeric_limits<double>::infinity()

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

GraphVertex::GraphVertex(pair<int,int> indices, int xsz, int ysz)
{
    m_X = indices.first; m_Y = indices.second;
    m_Xsz = xsz; m_Ysz = ysz;
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
    if((vertex.m_WayPtAssignment).size() > 0)
    {
        os << "\nWay Point Assignment: ";
        for(auto i: vertex.m_WayPtAssignment)
            os << i << " ";
        os << "\nPer Robot Costs: ";
        for(auto i: vertex.m_WayPtAssignmentCosts)
            os << i << " ";
        os << "\nLast Way Point Assigned: " << vertex.m_lastAssigned;
    }

    if((vertex.m_WayPtVisitation).size() > 0)
    {
        os << "\nWay Points Visited: ";
        for(auto i: vertex.m_WayPtVisitation)
            os << i << " ";
        os << "\nLast Way Point Visited: " << vertex.m_lastVisited;
    }
    os << "\nG-Value: " << vertex.m_gValue;
    return os;
}
