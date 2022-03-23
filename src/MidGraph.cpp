#include "MidGraph.hpp"
#include "ComparePriorityMid.cpp"

#include <vector>
#include <iostream>
#include <cmath>
#include <queue>
#include <unordered_map>
#define D_INF numeric_limits<double>::infinity()

using namespace std;

MidGraph::MidGraph(int numWayPts, vector<pair<int,int>> starts, vector<pair<int,int>> goals, vector<pair<int,int>> wayPts, 
    double* map, int collision_thresh, int x_size, int y_size)
{
	m_numWayPts = numWayPts; 
	m_starts = starts;
	m_goals = goals;
	m_wayPts = wayPts;
	m_map = map;
    m_CollThresh = collision_thresh;
    m_Xsz = x_size;
    m_Ysz = y_size;
	//cout << "Mid Graph is constructed" << endl;
}

MidGraph::~MidGraph()
{
	//cout << "Mid graph is destructed" << endl;
}

int MidGraph::MidHash(MidVertex* vertex)
{
    vector<bool> vec = vertex->m_WayPtVisitation;
    int num = 0;
    for(int i = 0; i < vec.size(); i++)
        num += vec[i]*pow(2,i);
    num += vertex->m_lastVisited * pow(2,vec.size());
    return num;
}

MidGraph::MidVertex* MidGraph::GetMidVertex(vector<int>& WayPtAssignment, int robot)
{
    vector<bool> wayPtsToVisit(m_numWayPts, 0);
    for(int i = 0; i < m_numWayPts; i++)
        if(WayPtAssignment[i] == robot)
            wayPtsToVisit[i] = 1;
    vector<LowGraph::LowVertex*> empty; 
    return(new MidVertex(wayPtsToVisit, m_numWayPts + 1, empty)); // goal for the mid-level search 
}

vector<MidGraph::MidVertex*> MidGraph::GetSuccessorsMid(MidVertex* vertex, MidVertex* end)
{
    vector<MidVertex*> succ; MidVertex* s;

    // [00,3] should not have successors; after reaching 3, do not generate any successors  
    if (vertex->m_lastVisited == m_numWayPts + 1)
    {
        return succ; 
    }

    for(int i = 0; i < m_numWayPts; i++)
    {
        if(vertex->m_WayPtVisitation[i] == 0 && end->m_WayPtVisitation[i] == 1) // going towards right here also 
        {
            vector<LowGraph::LowVertex*> empty;
            s = new MidVertex(vertex->m_WayPtVisitation, i+1, empty); // For mid level graph, omega becomes (i+1) - where is the robot currently at 
            s->m_WayPtVisitation[i] = 1;
            succ.push_back(s);
        }
    }

    if(vertex->m_lastVisited <= m_numWayPts)
    {
        vector<LowGraph::LowVertex*> empty;
        succ.push_back(new MidVertex(vertex->m_WayPtVisitation, m_numWayPts+1, empty));  // go directly to goal
    }

    return succ;
}

double MidGraph::MidSearchCost(vector<int>& WayPtAssignment, int robot)
{
    MidVertex* end = GetMidVertex(WayPtAssignment, robot);
    priority_queue<MidVertex*, vector<MidVertex*>, ComparePriorityMid> open;
    unordered_map<int, bool> closed;
    unordered_map<int, MidVertex*> vertices;

    double gVal_s; int hash_s;
    vector<MidVertex*> succ;
    vector<LowGraph::LowVertex*> empty;
    MidVertex* curr = new MidVertex(vector<bool>(m_numWayPts,0),0,empty);
    curr->m_parent = NULL; // Parent of start Node is set to NULL 
    curr->SetFValue(0,0);
    vertices[MidHash(curr)] = curr;
    open.push(curr);

    //bool goal_expanded = 0;
    MidVertex* goalNode;

    vector<MidVertex*> MidPath;

    while(!open.empty())
    {
        while(closed[MidHash(open.top())]){ open.pop();}
        curr = open.top(); open.pop();
        closed[MidHash(curr)] = 1;

        //cout << "\nMid Current:" << *curr << '\n';

        if(curr->m_WayPtVisitation == end->m_WayPtVisitation &&
            curr->m_lastVisited == end->m_lastVisited)
            {
                // goalNode = curr;
                // goal_expanded = 1;
                // continue;
                return curr->m_gValue; 
            } // only return the g-value 

        succ = GetSuccessorsMid(curr,end);
        for(MidVertex* s: succ)
        {
            hash_s = MidHash(s);
            if(!closed[hash_s])
            {
                
                pair<int,int> s_low, g_low;
                s_low = (curr->m_lastVisited == 0) ? m_starts[robot-1] : m_wayPts[curr->m_lastVisited-1];
                g_low = (s->m_lastVisited == m_numWayPts+1) ? m_goals[robot-1] : m_wayPts[s->m_lastVisited-1];
                LowGraph::LowGraph *GL = new LowGraph(m_map, m_CollThresh, m_Xsz, m_Ysz);
                //vector<LowGraph::LowVertex*> low_level_path = GL->LowSearch(s_low, g_low, robot); // I have to assign this low level path to Mid graph's node 
                double lowSearchCost = GL->LowSearchCost(s_low, g_low, robot);
                GL->~LowGraph();
                //double lowSearchCost = low_level_path.back()->m_gValue;
                //gVal_s = curr->m_gValue + LowSearch(curr, s, robot);
                gVal_s = curr->m_gValue + lowSearchCost;
                s->SetFValue(gVal_s,0);

                if(vertices[hash_s] == NULL) { vertices[hash_s] = s; }
                else { s = vertices[hash_s]; }
                
                if(gVal_s <= vertices[hash_s]->m_gValue) // by default equal if seen for the first time 
                {
                    s->SetFValue(gVal_s,0);
                    s->m_parent = vertices[MidHash(curr)];
                    //s->m_low_level_path = low_level_path; // Low level path from the parent to node 
                    open.push(s);
                }
                //cout << "\nMid Successor:" << *s << '\n';
            }
        }       
    }

    return D_INF;
}


vector<MidGraph::MidVertex*> MidGraph::MidSearch(vector<int>& WayPtAssignment, int robot)
{
    MidVertex* end = GetMidVertex(WayPtAssignment, robot);
    priority_queue<MidVertex*, vector<MidVertex*>, ComparePriorityMid> open;
    unordered_map<int, bool> closed;
    unordered_map<int, MidVertex*> vertices;

    double gVal_s; int hash_s;
    vector<MidVertex*> succ;
    vector<LowGraph::LowVertex*> empty;
    MidVertex* curr = new MidVertex(vector<bool>(m_numWayPts,0),0,empty);
    curr->m_parent = NULL; // Parent of start Node is set to NULL 
    curr->SetFValue(0,0);
    vertices[MidHash(curr)] = curr;
    open.push(curr);

    bool goal_expanded = 0;
    MidVertex* goalNode;

    vector<MidVertex*> MidPath;

    while(!open.empty() && !goal_expanded)
    {
        while(closed[MidHash(open.top())]){ open.pop();}
        curr = open.top(); open.pop();
        closed[MidHash(curr)] = 1;

        //cout << "\nMid Current:" << *curr << '\n';

        if(curr->m_WayPtVisitation == end->m_WayPtVisitation &&
            curr->m_lastVisited == end->m_lastVisited)
            {
                goalNode = curr;
                goal_expanded = 1;
                continue;
            } // only return the g-value 

        succ = GetSuccessorsMid(curr,end);
        for(MidVertex* s: succ)
        {
            hash_s = MidHash(s);
            if(!closed[hash_s])
            {
                
                pair<int,int> s_low, g_low;
                s_low = (curr->m_lastVisited == 0) ? m_starts[robot-1] : m_wayPts[curr->m_lastVisited-1];
                g_low = (s->m_lastVisited == m_numWayPts+1) ? m_goals[robot-1] : m_wayPts[s->m_lastVisited-1];
                LowGraph::LowGraph *GL = new LowGraph(m_map, m_CollThresh, m_Xsz, m_Ysz);
                vector<LowGraph::LowVertex*> low_level_path = GL->LowSearch(s_low, g_low, robot); // I have to assign this low level path to Mid graph's node 
                GL->~LowGraph();
                double lowSearchCost = low_level_path.back()->m_gValue;

                //gVal_s = curr->m_gValue + LowSearch(curr, s, robot);
                gVal_s = curr->m_gValue + lowSearchCost;
                s->SetFValue(gVal_s,0);

                if(vertices[hash_s] == NULL) { vertices[hash_s] = s; }
                else { s = vertices[hash_s]; }
                
                if(gVal_s <= vertices[hash_s]->m_gValue) // by default equal if seen for the first time 
                {
                    s->SetFValue(gVal_s,0);
                    s->m_parent = vertices[MidHash(curr)];
                    s->m_low_level_path = low_level_path; // Low level path from the parent to node 
                    open.push(s);
                }
                //cout << "\nMid Successor:" << *s << '\n';
            }
        }       
    }

    if (goal_expanded)
    {
        MidPath.push_back(goalNode);
        while (goalNode->m_parent) // start node won't have a parent 
        {
            goalNode = goalNode->m_parent;
            MidPath.push_back(goalNode);
        }
        reverse(MidPath.begin(), MidPath.end());
        return MidPath;
    }

    return MidPath;
    //return D_INF;
}

// int main()
// {

// 	vector<pair<int,int>> starts, goals, wayPts;

// 	starts.push_back({0,0});
// 	starts.push_back({0,0});

// 	goals.push_back({0,0});
// 	goals.push_back({0,0});

// 	wayPts.push_back({3,7});
// 	wayPts.push_back({7,7});

// 	int M = wayPts.size();
// 	int N = starts.size();

// 	vector<vector<int>> map(8, vector<int>(8,0));

// 	MidGraph::MidGraph *GM = new MidGraph(M, starts, goals, wayPts, map);
// 	GM->~MidGraph();
// 	return 0;
// }
