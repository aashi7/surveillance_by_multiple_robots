#include "GraphVertex.hpp"

using namespace std;

class ComparePriority
{
	public:
		bool operator () (GraphVertex* a, GraphVertex* b) const
		{
			return (a->m_fValue > b->m_fValue);
		}
};