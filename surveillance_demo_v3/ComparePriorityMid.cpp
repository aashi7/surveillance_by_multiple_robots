#include "MidGraph.hpp"

using namespace std;

class ComparePriorityMid
{
	public:
		bool operator () (MidGraph::MidVertex* a, MidGraph::MidVertex* b) const
		{
			return (a->m_fValue > b->m_fValue);
		}
};