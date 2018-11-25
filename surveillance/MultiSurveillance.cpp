#include <queue>
#include <unordered_map>
#include <limits>

#include "MultiSurveillance.hpp"
#include "ComparePriority.cpp"
#include "HashTop.cpp"
#include "HashMid.cpp"

#define D_INF numeric_limits<double>::infinity()
using namespace std;

MultiSurveillance::MultiSurveillance(int M, int N, 
    vector<pair<int,int>> starts, vector<pair<int,int>> goals)
{
	m_numWayPts = M;
	m_numRobots = N;
	m_starts = starts;
	m_goals = goals;
}

MultiSurveillance::~MultiSurveillance(){}

vector<GraphVertex*> MultiSurveillance::GetSuccessorsTop(GraphVertex* vertex)
{
    vector<GraphVertex*> succ; GraphVertex* s;
    int nextAssign = vertex->m_lastAssigned + 1;
    if(nextAssign <= m_numWayPts)
    {
        for(int i = 0; i < m_numRobots; i++)
        {
            s = new GraphVertex(vertex->m_WayPtAssignment, nextAssign,
                vertex->m_WayPtAssignmentCosts, vertex);
            s->m_WayPtAssignment[nextAssign-1] = i+1;
            s->m_WayPtAssignmentCosts[i] = MidSearch(vertex,i+1);
            succ.push_back(s);
        }

    }
    return succ;
}

GraphVertex* MultiSurveillance::TopSearch()
{
    priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority> open;
    unordered_map<vector<int>, bool, HashTop> closed;

    double gVal, gVal_s; int robot_s;
    vector<GraphVertex*> succ;
    GraphVertex* curr = new GraphVertex(vector<int>(m_numWayPts,0), 0,
                                   vector<double>(m_numRobots,0), NULL);
    curr->SetFValue(0,0);
       
    open.push(curr);
    while(!open.empty())
    {
        while(closed[(open.top())->m_WayPtAssignment]){open.pop();}
        curr = open.top(); open.pop();
        closed[curr->m_WayPtAssignment] = 1;
        gVal = curr->m_gValue;

        if(curr->m_lastAssigned == m_numWayPts){ return curr; }

        succ = GetSuccessorsTop(curr);
        for(GraphVertex* s: succ)
        {
            robot_s = s->m_WayPtAssignment[s->m_lastAssigned-1] - 1;
            gVal_s = gVal + (s->m_WayPtAssignmentCosts[robot_s] - 
                          curr->m_WayPtAssignmentCosts[robot_s]);
            s->SetFValue(gVal_s,0);
            open.push(s);
        }

    }
    return NULL;
}

vector<GraphVertex*> MultiSurveillance::GetSuccessorsMid(GraphVertex* vertex, GraphVertex* end)
{
    vector<GraphVertex*> succ; GraphVertex* s;
    for(int i = 0; i < m_numWayPts; i++)
    {
        if(vertex->m_WayPtVisitation[i] == 0 && end->m_WayPtVisitation[i] == 1)
        {
            s = new GraphVertex(vertex->m_WayPtVisitation, i+1);
            s->m_WayPtVisitation[i] = 1;
            succ.push_back(s);
        }
    }

    if(vertex->m_lastVisited <= m_numWayPts)
        succ.push_back(new GraphVertex(vertex->m_WayPtVisitation, m_numWayPts+1));

    return succ;
}

GraphVertex* MultiSurveillance::GetMidVertex(GraphVertex* goal, int robot)
{
    vector<bool> wayPtsToVisit(m_numWayPts,0);
    for(int i = 0; i < m_numWayPts; i++)
        if(goal->m_WayPtAssignment[i] == robot) 
            wayPtsToVisit[i] = 1;
    return(new GraphVertex(wayPtsToVisit, m_numWayPts+1));
}

double MultiSurveillance::MidSearch(GraphVertex* goal, int robot)
{
    GraphVertex* end = GetMidVertex(goal,robot);
    priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority> open;
    unordered_map<vector<bool>, bool, HashMid> closed;

    double gVal, gVal_s;
    vector<GraphVertex*> succ;
    GraphVertex* curr = new GraphVertex(vector<bool>(m_numWayPts,0), 0);
    curr->SetFValue(0,0);
       
    open.push(curr);
    while(!open.empty())
    {
        while(closed[(open.top())->m_WayPtVisitation]){open.pop();}
        curr = open.top(); open.pop();
        closed[curr->m_WayPtVisitation] = 1;
        gVal = curr->m_gValue;

        if(curr->m_WayPtVisitation == end->m_WayPtVisitation &&
            curr->m_lastVisited == end->m_lastVisited){ return curr->m_gValue; }

        succ = GetSuccessorsMid(curr,end);
        for(GraphVertex* s: succ)
        {
            gVal_s = gVal + LowSearch(curr, s, robot);
            s->SetFValue(gVal_s,0);
            open.push(s);
        }

    }
    return D_INF;
}

double MultiSurveillance::LowSearch(GraphVertex* start, GraphVertex* goal, int robot)
{
    return 0;
}

vector<GraphVertex*> MultiSurveillance::GetSuccessorsLow(GraphVertex* vertex)
{
    return vector<GraphVertex*>();
}