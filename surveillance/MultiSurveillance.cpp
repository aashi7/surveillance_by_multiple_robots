#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <cmath>

#include <stdlib.h>
#include <time.h>
#include <boost/functional/hash.hpp>

#include "MultiSurveillance.hpp"
#include "ComparePriority.cpp"

#define D_INF numeric_limits<double>::infinity()
using namespace std;


/* *************************** */
/*      CONSTRUCTOR           */
/* ************************* */

MultiSurveillance::MultiSurveillance(int M, int N, vector<pair<int,int>> wayPts,
    vector<pair<int,int>> starts, vector<pair<int,int>> goals,
    double* costmap, double collision_thresh, int xsz, int ysz)
{
	m_numWayPts = M;
	m_numRobots = N;
    m_wayPts = wayPts;
	m_starts = starts;
	m_goals = goals;
    m_costmap = costmap;
    m_collThreshold = collision_thresh;
    m_mapXsize = xsz;
    m_mapYsize = ysz;
}

/* *************************** */
/*       DESTRUCTOR           */
/* ************************* */

MultiSurveillance::~MultiSurveillance(){}


/* *************************** */
/* TOP LEVEL SEARCH FUNCTIONS */
/* ************************* */


double MultiSurveillance::SearchCost(GraphVertex* reachedGoal)
{
    return ( (reachedGoal == NULL) ? D_INF : (reachedGoal->m_gValue) );
}

int MultiSurveillance::TopHash(GraphVertex* vertex)
{
    vector<int> vec = vertex->m_WayPtAssignment;
    return boost::hash_range(vec.begin(), vec.end());
}

void MultiSurveillance::TopInitOpen(GraphVertex* current, unordered_map<int, GraphVertex*>* vertices, 
    priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority>* open)
{
    current = new GraphVertex(vector<int>(m_numWayPts,0), 0,
                           vector<double>(m_numRobots,0), NULL);
    current->SetFValue(0,0);
    (*vertices)[TopHash(current)] = current;
    (*open).push(current);
}

bool MultiSurveillance::IsTopGoal(GraphVertex* vertex)
{
    return (vertex->m_lastAssigned == m_numWayPts);
}

vector<GraphVertex*> MultiSurveillance::TopSuccessors(GraphVertex* vertex)
{
    vector<GraphVertex*> succ; GraphVertex *s;
    int nextAssign = vertex->m_lastAssigned + 1;

    if(nextAssign <= m_numWayPts)
    {
        for(int i = 0; i < m_numRobots; i++)
        {
            s = new GraphVertex(vertex->m_WayPtAssignment, nextAssign,
                vertex->m_WayPtAssignmentCosts, vertex);
            s->m_WayPtAssignment[nextAssign-1] = i+1;
            s->m_WayPtAssignmentCosts[i] = SearchCost(MidSearch(s,i+1));
            succ.push_back(s);
        }
    }
    return succ;
}

double MultiSurveillance::TopPathCost(GraphVertex* current, GraphVertex* successor)
{
    int robot_s = successor->m_WayPtAssignment[successor->m_lastAssigned-1];
    double gVal_s = current->m_gValue + (successor->m_WayPtAssignmentCosts[robot_s-1] - 
                  current->m_WayPtAssignmentCosts[robot_s-1]);

    // In case some robot has not been assigned any waypoint,
    // add the cost for that robot to its goal.
    if(successor->m_lastAssigned == m_numWayPts)
    {
        for(int i = 0; i < m_numRobots; i++)
            if(find(successor->m_WayPtAssignment.begin(), 
                successor->m_WayPtAssignment.end(), i+1)
                == successor->m_WayPtAssignment.end())
                gVal_s += SearchCost(MidSearch(successor,i+1));
    }
    return gVal_s;
}

GraphVertex* MultiSurveillance::TopSearch()
{
    priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority> open;
    unordered_map<int, bool> closed;
    unordered_map<int, GraphVertex*> vertices;

    double gVal_s; int robot_s, hash_s;
    GraphVertex* curr; vector<GraphVertex*> succ;
    TopInitOpen(curr, &vertices, &open);
    while(!open.empty())
    {
        while(closed[TopHash(open.top())]){open.pop();}
        curr = open.top(); open.pop();
        closed[TopHash(curr)] = 1;

        //cout << "\nTop Current:" << *curr << '\n';

        if(IsTopGoal(curr)){ return curr; }

        succ = TopSuccessors(curr);
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
                //cout << "\nTop Successor:" << *s << '\n';
            }
        }

    }
    return NULL;
}


/* *************************** */
/* MID LEVEL SEARCH FUNCTIONS */
/* ************************* */


GraphVertex* MultiSurveillance::MidVertex(GraphVertex* goal, int robot)
{
    vector<bool> wayPtsToVisit(m_numWayPts,0);
    for(int i = 0; i < m_numWayPts; i++)
        if(goal->m_WayPtAssignment[i] == robot) 
            wayPtsToVisit[i] = 1;
    return(new GraphVertex(wayPtsToVisit, m_numWayPts+1));
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

void MultiSurveillance::MidInitOpen(GraphVertex* current, unordered_map<int, GraphVertex*>* vertices, 
    priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority>* open)
{
    current = new GraphVertex(vector<bool>(m_numWayPts,0), 0);
    current->SetFValue(0,0);
    current->m_parent = NULL;
    current->m_lowPtr = NULL;
    (*vertices)[MidHash(current)] = current;
    (*open).push(current);
}

bool MultiSurveillance::IsMidGoal(GraphVertex* vertex, GraphVertex* goal)
{
    return (vertex->m_WayPtVisitation == goal->m_WayPtVisitation &&
            vertex->m_lastVisited == goal->m_lastVisited);
}

vector<GraphVertex*> MultiSurveillance::MidSuccessors(GraphVertex* vertex, GraphVertex* end)
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

pair<double,GraphVertex*> MultiSurveillance::MidPath(GraphVertex* current, GraphVertex* successor, int robot)
{
    GraphVertex* lowSearch = LowSearch(current, successor, robot);
    return pair<double,GraphVertex*>((current->m_gValue + SearchCost(lowSearch)), lowSearch);
}

GraphVertex* MultiSurveillance::MidSearch(GraphVertex* goal, int robot)
{
    GraphVertex* end = MidVertex(goal,robot);
    //cout << "\nMid Goal:" << *end << '\n';

    priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority> open;
    unordered_map<int, bool> closed;
    unordered_map<int, GraphVertex*> vertices;

    double gVal_s; int hash_s;
    GraphVertex *curr, *lowSearch; vector<GraphVertex*> succ;
    MidInitOpen(curr, &vertices, &open);
    while(!open.empty())
    {
        while(closed[MidHash(open.top())]){ open.pop();}
        curr = open.top(); open.pop();
        closed[MidHash(curr)] = 1;

        //cout << "\nMid Current:" << *curr << '\n';

        if(IsMidGoal(curr,end)){ return curr; }

        succ = MidSuccessors(curr,end);
        for(GraphVertex* s: succ)
        {
            hash_s = MidHash(s);
            if(!closed[hash_s])
            {
                tie(gVal_s, lowSearch) = MidPath(curr, s, robot);
                s->SetFValue(gVal_s,0);

                if(vertices[hash_s] == NULL) { vertices[hash_s] = s; }
                else { s = vertices[hash_s]; }
                
                if(gVal_s <= vertices[hash_s]->m_gValue)
                {
                    s->SetFValue(gVal_s,0);
                    s->m_parent = vertices[MidHash(curr)];
                    s->m_lowPtr = lowSearch;
                    open.push(s);
                }
                //cout << "\nMid Successor:" << *s << '\n';
            }
        }
    }
    
    return NULL;
}


/* *************************** */
/* LOW LEVEL SEARCH FUNCTIONS */
/* ************************* */

int MultiSurveillance::LowHash(GraphVertex* vertex)
{
    return ((vertex->m_Y)*(vertex->m_Xsz) + (vertex->m_X));
}

void MultiSurveillance::LowInitOpen(pair<int,int> start, GraphVertex* current, unordered_map<int, GraphVertex*>* vertices, 
    priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority>* open)
{
    current = new GraphVertex(start, m_mapXsize, m_mapYsize);
    current->SetFValue(0,0);
    current->m_parent = NULL;
    (*vertices)[LowHash(current)] = current;
    (*open).push(current);
}

bool MultiSurveillance::IsLowGoal(GraphVertex* vertex, pair<int,int> goal)
{
    return (vertex->m_X == goal.first && vertex->m_Y == goal.second);
}

vector<GraphVertex*> MultiSurveillance::LowSuccessors(GraphVertex* vertex)
{
    vector<GraphVertex*> succ;
    GraphVertex* s; int n_X, n_Y;
    vector<int> dX{-1, -1, 1,  1,  0, 0, 1, -1};
    vector<int> dY{-1,  1, 1, -1, -1, 1, 0,  0};
    for(int i = 0; i < dX.size(); i++)
    {
        n_X = vertex->m_X + dX[i];
        n_Y = vertex->m_Y + dY[i];
        if(n_X < vertex->m_Xsz && n_X >= 0 && 
           n_Y < vertex->m_Ysz && n_Y >= 0 &&
           m_costmap[n_Y*(vertex->m_Xsz)+n_X] < m_collThreshold)
        {
            s = new GraphVertex(pair<int,int>(n_X,n_Y), 
                           vertex->m_Xsz, vertex->m_Ysz);
            succ.push_back(s);
        }
    }
    return succ;
}

double MultiSurveillance::LowPathCost(GraphVertex* current, GraphVertex* successor)
{
    return (current->m_gValue + m_costmap[LowHash(current)]);
}

GraphVertex* MultiSurveillance::LowSearch(GraphVertex* start, GraphVertex* goal, int robot)
{
    pair<int,int> s, g;
    s = (start->m_lastVisited == 0) ? m_starts[robot-1] : m_wayPts[start->m_lastVisited-1];
    g = (goal->m_lastVisited == m_numWayPts+1) ? m_goals[robot-1] : m_wayPts[goal->m_lastVisited-1];

    // 1-indexed from matlab => 0-indexing (required for c++)
    s.first = s.first - 1; s.second = s.second - 1; g.first = g.first - 1; g.second = g.second - 1;

    // double eucDist = sqrt(pow(s.first-g.first,2) + pow(s.second-g.second,2));
    // GraphVertex* reachedGoal = new GraphVertex(); reachedGoal->SetFValue(eucDist,0);
    // return reachedGoal;

    priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority> open;
    unordered_map<int, bool> closed;
    unordered_map<int, GraphVertex*> vertices;

    double gVal_s; int hash_s;
    GraphVertex* curr; vector<GraphVertex*> succ;
    LowInitOpen(s, curr, &vertices, &open);
    while(!open.empty())
    {
        while(closed[LowHash(open.top())]){ open.pop();}
        curr = open.top(); open.pop();
        closed[LowHash(curr)] = 1;

        if(IsLowGoal(curr,g)){ return curr; }

        succ = LowSuccessors(curr);
        for(GraphVertex* s: succ)
        {
            hash_s = LowHash(s);
            if(!closed[hash_s])
            {
                gVal_s = LowPathCost(curr, s);
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
    
    return NULL;
}

vector<pair<int,int>> MultiSurveillance::BackTrackLowPlan(GraphVertex* midSearchPtr)
{
    if(midSearchPtr == NULL){ return vector<pair<int,int>>();}
    vector<pair<int,int>> backPath;
    GraphVertex* lowSearchPtr = midSearchPtr->m_lowPtr;
    while(lowSearchPtr != NULL)
    {
        backPath.push_back(pair<int,int>(lowSearchPtr->m_X,lowSearchPtr->m_Y));
        lowSearchPtr = lowSearchPtr->m_parent;
    }

    return backPath;
}

pair<double***,int*> MultiSurveillance::RunPlan(bool isRandom)
{
    clock_t begin_time = clock();

    vector<vector<pair<int,int>>> finalPlan(m_numRobots);
    vector<pair<int,int>> robotLowPlan, robotPartPlan;
    vector<int> finalPlanLengths(m_numRobots,0);
    GraphVertex *midSearchPtr, *topSearchPtr;
    if(!isRandom){ topSearchPtr = TopSearch(); }
    else{ topSearchPtr = RandomAssignment(); }
    double gValue = 0;
    for(int i = 0 ; i < m_numRobots; i++)
    {
        robotLowPlan = vector<pair<int,int>>();
        midSearchPtr = MidSearch(topSearchPtr,i+1);
        gValue += midSearchPtr->m_gValue;
        while(midSearchPtr != NULL)
        {
            robotPartPlan = BackTrackLowPlan(midSearchPtr);
            robotLowPlan.insert(robotLowPlan.end(), robotPartPlan.begin(), robotPartPlan.end());
            midSearchPtr = midSearchPtr->m_parent;
        }
        reverse(robotLowPlan.begin(), robotLowPlan.end());
        finalPlan[i] = robotLowPlan;
        finalPlanLengths[i] = finalPlan[i].size();
    }

    m_plantime = double(clock() - begin_time)/CLOCKS_PER_SEC;
    m_plancost = gValue;

    double*** finalPlanPtr = (double***) malloc(m_numRobots*sizeof(double**));
    for(int i = 0; i < m_numRobots; i++)
    {
        finalPlanPtr[i] = (double**) malloc(finalPlanLengths[i]*sizeof(double*));
        for(int j = 0; j < finalPlanLengths[i]; j++)
        {
            finalPlanPtr[i][j] = (double*) malloc(2*sizeof(double));
            finalPlanPtr[i][j][0] = (double) finalPlan[i][j].first;
            finalPlanPtr[i][j][1] = (double) finalPlan[i][j].second;
        }
    }

    int* finalPlanLengthsPtr = (int*) malloc(m_numRobots*sizeof(int));
    for(int i = 0; i < m_numRobots; i++)
        finalPlanLengthsPtr[i] = finalPlanLengths[i];

    return(pair<double***,int*>(finalPlanPtr,finalPlanLengthsPtr));
}

GraphVertex* MultiSurveillance::RandomAssignment()
{
    GraphVertex* randomAssign = new GraphVertex(vector<int>(m_numWayPts,0), 0,
                           vector<double>(m_numRobots,0), NULL);
    for(int i = 0; i < m_numWayPts; i++)
        randomAssign->m_WayPtAssignment[i] = (rand()%m_numRobots) + 1;

    return randomAssign;
}