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
    bool operator () (GraphVertex* a, GraphVertex* b) const {
        return (a->m_fValue > b->m_fValue);}
};