#include "TopGraph.hpp"

using namespace std;

class ComparePriorityTop
{
	public:
		bool operator () (TopGraph::TopVertex* a, TopGraph::TopVertex* b) const
		{
			return (a->m_fValue > b->m_fValue);
		}
};