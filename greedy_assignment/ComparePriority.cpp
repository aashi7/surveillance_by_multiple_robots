#ifndef COMPAREPRIORITY_CPP
#define COMPAREPRIORITY_CPP

#include "GraphVertex.hpp"

using namespace std;

/* Priority of vertex in graph in Dijkstra search
   (according to g-value), using a min-heap 
   Inputs: 2 Graph Vertices 
   Output: boolean after comparing priorities (pathcost + heuristic)
*/
class ComparePriority
{
public:
    typedef std::shared_ptr<GraphVertex> GraphVertexPtr_t;
    bool operator () (GraphVertexPtr_t a, GraphVertexPtr_t b) const {
        return (a->m_fValue > b->m_fValue);}
};

#endif