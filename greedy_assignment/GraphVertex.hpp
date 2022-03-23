#ifndef GRAPHVERTEX_HPP
#define GRAPHVERTEX_HPP

#include <vector>
#include <iostream>
#include <memory>

using namespace std;

/* Class for vertex in graphs */
class GraphVertex
{
public:
    typedef std::shared_ptr<GraphVertex> GraphVertexPtr_t;
    GraphVertex();
    GraphVertex(vector<int> perWayPtRobots, int lastWayPt, 
                vector<double> perRobotCosts, GraphVertexPtr_t parent);
    GraphVertex(vector<bool> wayPtsDone, int lastWayPt);
    GraphVertex(pair<int,int> indices, int xsz, int ysz);
    ~GraphVertex();

    void SetFValue(double gVal, double hVal);
    friend ostream& operator<<(ostream& os, const GraphVertex& vertex);

    // Vertex for top level graph - search top graph for final answer
    vector<int> m_WayPtAssignment;         /*  M vector:   each waypoint is assigned to a robot    */
    int m_lastAssigned;                    /*  int id:     last waypoint to be assigned            */
    vector<double> m_WayPtAssignmentCosts; /*  N vector:   cost of assigning waypts to each robot  */


    // Vertex for mid level graph - one mid graph per robot
    vector<bool> m_WayPtVisitation;        /*  M vector:   which waypoints have been visited?      */
    int m_lastVisited;                     /*  int id:     which waypoint was visited last?        */
    GraphVertexPtr_t m_lowPtr;             /*  pointer:    to low search goal (which corresponds
                                                           to edge from parent)                    */


    // Vertex for low level graph
    int m_X; int m_Y;                       /*  x and y index into grid                             */
    int m_Xsz; int m_Ysz;                   /*  map size                                            */


    GraphVertexPtr_t m_parent;             /*  vertex parent   */
    double m_gValue;                       /*  cost from start */
    double m_hValue;                       /*  heuristic value */
    double m_fValue;                       /*  priority value  */
	

};

#endif