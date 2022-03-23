#include "LowGraph.hpp"

using namespace std;

class ComparePriorityLow
{
	public:
		bool operator () (LowGraph::LowVertex* a, LowGraph::LowVertex* b) const
		{
			return (a->m_fValue > b->m_fValue);
		}
};