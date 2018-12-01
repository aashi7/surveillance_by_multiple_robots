#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <cmath>
//#include <boost/functional/hash.hpp>

#include "MultiSurveillance.hpp"
#include "ComparePriority.cpp"

#define D_INF numeric_limits<double>::infinity()
#define MAPINDEX(X,Y,XSIZE,YSIZE) ((Y-1)*XSIZE + (X-1))

using namespace std;

MultiSurveillance::MultiSurveillance(int M, int N, vector<pair<int,int>> wayPts,
	vector<pair<int,int>> starts, vector<pair<int,int>> goals, vector<vector<int>>& map, int xsz, int ysz)
{
	m_numWayPts = M;
	m_numRobots = N;
	m_wayPts = wayPts;
	m_starts = starts;
	m_goals = goals;
	m_map = map;
	m_mapXsize = xsz;
	m_mapYsize = ysz;
}

MultiSurveillance::~MultiSurveillance(){}

int MultiSurveillance::TopHash(GraphVertex* vertex)
{
	vector<int> vec = vertex->m_WayPtAssignment;
    // write own hash function top graph, boost functional hpp can't be compiled through MATLAB
    //return boost::hash_range(vec.begin(), vec.end());
    
    int seed = vec.size();
    for(auto& i : vec) 
    {
        seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
     return seed;
}


vector<GraphVertex*> MultiSurveillance::GetSuccessorsTop(GraphVertex* vertex, unordered_map<int, GraphVertex*> vertices)
{
	vector<GraphVertex*> succ;
	GraphVertex *s, *parent = vertices[TopHash(vertex)]; // parent is simply the vertex 
	int nextAssign = vertex->m_lastAssigned + 1; // going towards right 

	if (nextAssign <= m_numWayPts)
	{
		for (int i = 0; i < m_numRobots; i++)
		{
			s = new GraphVertex(vertex->m_WayPtAssignment, nextAssign, vertex->m_WayPtAssignmentCosts, parent);
			s->m_WayPtAssignment[nextAssign-1] = i+1; // assigned the robot id at index nextAssign-1
			s->m_WayPtAssignmentCosts[i] = MidSearch(s,i+1);
			//cout << " Mid search: " << MidSearch(s,i+1) << endl;
			succ.push_back(s);
		}
	}

	return succ;
}

double MultiSurveillance::TopPathCost(GraphVertex* current, GraphVertex* successor) // MidSearch returns the cost of edge  - Handles the edge case when robot has to plan from start to goal with no waypoint visitation  
{
	int robot_s = successor->m_WayPtAssignment[successor->m_lastAssigned-1]; // same as robot_id
	double gVal_s = current->m_gValue + (successor->m_WayPtAssignmentCosts[robot_s-1] - current->m_WayPtAssignmentCosts[robot_s-1]);
	//cout << robot_s << " " << gVal_s << endl;
	// Edge case: When robot has not been assigned any waypoint
	if (successor->m_lastAssigned == m_numWayPts) // When all the waypoints are assigned 
	{
		for (int i = 0; i < m_numRobots; i++)
		if (find(successor->m_WayPtAssignment.begin(), successor->m_WayPtAssignment.end(), i+1) == successor->m_WayPtAssignment.end())
			gVal_s += MidSearch(successor, i+1);
	}

	return gVal_s;
}

GraphVertex* MultiSurveillance::TopSearch()
{
	priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority> open;
	unordered_map<int, bool> closed;
	unordered_map<int, GraphVertex*> vertices;

	double gVal_s; int robot_s, hash_s; // what does robot_s do?
	vector<GraphVertex*> succ;

	GraphVertex* curr = new GraphVertex(vector<int>(m_numWayPts,0), 0, vector<double>(m_numRobots,0), NULL);
	curr->SetFValue(0,0);
	vertices[TopHash(curr)] = curr;
	open.push(curr);

	while(!open.empty())
	{
		while(closed[TopHash(open.top())]) {open.pop();}
		curr = open.top(); open.pop();
		closed[TopHash(curr)] = 1;

		if(curr->m_lastAssigned == m_numWayPts) {return curr ;} // Why it isn't m_numWaypts - 1

		succ = GetSuccessorsTop(curr, vertices);

	    for(GraphVertex* s: succ)
	    {
	        hash_s = TopHash(s);
	        if(!closed[hash_s])
	        {
	            gVal_s = TopPathCost(curr,s);
	            s->SetFValue(gVal_s,0);

	            if(vertices[hash_s] == NULL) { vertices[hash_s] = s; }
	            else { s = vertices[hash_s]; }
	            
	            if(gVal_s <= vertices[hash_s]->m_gValue)
	            {
	                s->SetFValue(gVal_s,0);
	                s->m_parent = vertices[TopHash(curr)];
	                open.push(s);
	            }

	            cout << "\nTop Successor:" << *s << '\n';
	        }
	    }	

	}

	return NULL;
}

vector<GraphVertex*> MultiSurveillance::GetSuccessorsMid(GraphVertex* vertex, GraphVertex* end)
{
    vector<GraphVertex*> succ; GraphVertex* s;
    for(int i = 0; i < m_numWayPts; i++)
    {
        if(vertex->m_WayPtVisitation[i] == 0 && end->m_WayPtVisitation[i] == 1) // going towards right here also 
        {
            s = new GraphVertex(vertex->m_WayPtVisitation, i+1); // For mid level graph, omega becomes (i+1) - where is the robot currently at 
            s->m_WayPtVisitation[i] = 1;
            succ.push_back(s);
        }
    }

    if(vertex->m_lastVisited <= m_numWayPts)
        succ.push_back(new GraphVertex(vertex->m_WayPtVisitation, m_numWayPts+1));  // go directly to goal

    return succ;
}

GraphVertex* MultiSurveillance::GetMidVertex(GraphVertex* goal, int robot)
{
	vector<bool> wayPtsToVisit(m_numWayPts, 0);
	for(int i = 0; i < m_numWayPts; i++)
		if(goal->m_WayPtAssignment[i] == robot)
			wayPtsToVisit[i] = 1;
	return(new GraphVertex(wayPtsToVisit, m_numWayPts + 1)); // goal for the mid-level search 
}

int MultiSurveillance::MidHash(GraphVertex* vertex)
{
    vector<bool> vec = vertex->m_WayPtVisitation;
    int num = 0;
    for(int i = 0; i < vec.size(); i++)
        num += vec[i]*pow(2,i);
    num += vertex->m_lastVisited * pow(2,vec.size());
    return num;
}

double MultiSurveillance::MidSearch(GraphVertex* goal, int robot)
{
	GraphVertex* end = GetMidVertex(goal, robot);
	priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority> open;
	unordered_map<int, bool> closed;
	unordered_map<int, GraphVertex*> vertices;

	double gVal_s; int hash_s;
	vector<GraphVertex*> succ;
	GraphVertex* curr = new GraphVertex(vector<bool>(m_numWayPts,0),0);
	curr->SetFValue(0,0);
	vertices[MidHash(curr)] = curr;
	open.push(curr);

	while(!open.empty())
	{
        while(closed[MidHash(open.top())]){ open.pop();}
        curr = open.top(); open.pop();
        closed[MidHash(curr)] = 1;

        //cout << "\nMid Current:" << *curr << '\n';

        if(curr->m_WayPtVisitation == end->m_WayPtVisitation &&
            curr->m_lastVisited == end->m_lastVisited){ return curr->m_gValue; } // only return the g-value 

        succ = GetSuccessorsMid(curr,end);
        for(GraphVertex* s: succ)
        {
            hash_s = MidHash(s);
            if(!closed[hash_s])
            {
                gVal_s = curr->m_gValue + LowSearch(curr, s, robot);
                s->SetFValue(gVal_s,0);

                if(vertices[hash_s] == NULL) { vertices[hash_s] = s; }
                else { s = vertices[hash_s]; }
                
                if(gVal_s <= vertices[hash_s]->m_gValue) // by default equal if seen for the first time 
                {
                    s->SetFValue(gVal_s,0);
                    s->m_parent = vertices[MidHash(curr)];
                    open.push(s);
                }
                //cout << "\nMid Successor:" << *s << '\n';
            }
        }		
	}

	return D_INF;
}

int MultiSurveillance::LowHash(GraphVertex* vertex)
{
    return ((vertex->m_Y)*(vertex->m_Xsz) + (vertex->m_X));
}

double MultiSurveillance::LowSearch(GraphVertex* start, GraphVertex* goal, int robot)
{
    pair<int,int> s, g;
    s = (start->m_lastVisited == 0) ? m_starts[robot-1] : m_wayPts[start->m_lastVisited-1];
    g = (goal->m_lastVisited == m_numWayPts+1) ? m_goals[robot-1] : m_wayPts[goal->m_lastVisited-1];

    // double eucDist = sqrt(pow(s.first-g.first,2) + pow(s.second-g.second,2));
    // return eucDist;

    priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority> open;
    unordered_map<int, bool> closed;
    unordered_map<int, GraphVertex*> vertices;

    double gVal_s; int hash_s;
    vector<GraphVertex*> succ;
    GraphVertex* curr = new GraphVertex(s, m_mapXsize, m_mapYsize);
    curr->SetFValue(0,0);
    vertices[LowHash(curr)] = curr;
    open.push(curr);
    while(!open.empty())
    {
        while(closed[LowHash(open.top())]){ open.pop();}
        curr = open.top(); open.pop();
        closed[LowHash(curr)] = 1;

        if(curr->m_X == g.first && curr->m_Y == g.second){ return curr->m_gValue; }

        succ = GetSuccessorsLow(curr);

        gVal_s = curr->m_gValue + 1; // move in a 8-connected grid

        //gVal_s = curr->m_gValue + m_costmap[LowHash(curr)]; // Infinite cost for blocked cells    if (m_costmap[LowHash(curr)] == 1) but m_costmap would be a 2D matrix - isn't it? Does linear indexing work?
        for(GraphVertex* s: succ)
        {
            hash_s = LowHash(s);
            if(!closed[hash_s])
            {
                s->SetFValue(gVal_s,0);

                if(vertices[hash_s] == NULL) { vertices[hash_s] = s; }
                else { s = vertices[hash_s]; }
                
                if(gVal_s <= vertices[hash_s]->m_gValue)
                {
                    s->SetFValue(gVal_s,0);
                    s->m_parent = vertices[LowHash(curr)];
                    open.push(s);
                }
            }
        }
    }
    
    return D_INF;
}

vector<GraphVertex*> MultiSurveillance::GetSuccessorsLow(GraphVertex* vertex)
{
    vector<GraphVertex*> succ;
    GraphVertex* s; int n_X, n_Y;
    vector<int> dX{-1, -1, -1,  0,  0,  1, 1, 1};
    vector<int> dY{-1,  0,  1, -1,  1, -1, 0, 1};
    for(int i = 0; i < dX.size(); i++)
    {
        n_X = vertex->m_X + dX[i];
        n_Y = vertex->m_Y + dY[i];
        if(n_X < vertex->m_Xsz && n_X >= 0 && 
           n_Y < vertex->m_Ysz && n_Y >= 0 && m_map[n_X][n_Y] == 0) // collision-checking done here 
        {
            s = new GraphVertex(pair<int,int>(n_X,n_Y), 
                           vertex->m_Xsz, vertex->m_Ysz);
            succ.push_back(s);
        }
    }
    return succ;
}
