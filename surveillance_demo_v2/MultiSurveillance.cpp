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

    //cout << "Expanding top successor: " << *parent << endl;
    if (nextAssign <= m_numWayPts)
    {
        for (int i = 0; i < m_numRobots; i++)
        {
            vector<GraphVertex*> empty;
            s = new GraphVertex(vertex->m_WayPtAssignment, nextAssign, vertex->m_WayPtAssignmentCosts, parent, empty);
            s->m_WayPtAssignment[nextAssign-1] = i+1; // assigned the robot id at index nextAssign-1

            vector<GraphVertex*> mid_level_path = MidSearch(s,i+1);
            double MidCost = mid_level_path.back()->m_gValue;

            s->m_WayPtAssignmentCosts[i] = MidCost;
            //s->m_WayPtAssignmentCosts[i] = MidSearch(s,i+1);
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
        {

            vector<GraphVertex*> mid_level_path = MidSearch(successor, i+1);

            double MidSearchCost = mid_level_path.back()->m_gValue;
            //gVal_s += MidSearch(successor, i+1);
            gVal_s += MidSearchCost;
        }
    }

    return gVal_s;
}

vector<GraphVertex*> MultiSurveillance::TopSearch()
{
    priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority> open;
    unordered_map<int, bool> closed;
    unordered_map<int, GraphVertex*> vertices;

    double gVal_s; int robot_s, hash_s; // what does robot_s do?
    vector<GraphVertex*> succ;

    vector<GraphVertex*> empty; 
    GraphVertex* curr = new GraphVertex(vector<int>(m_numWayPts,0), 0, vector<double>(m_numRobots,0), NULL, empty);
    curr->m_parent = NULL;
    curr->SetFValue(0,0);
    vertices[TopHash(curr)] = curr;
    open.push(curr);

    vector<GraphVertex*> TopPath;
    bool goal_expanded = 0;
    GraphVertex* goalNode; 

    while(!open.empty() && !goal_expanded)
    {
        while(closed[TopHash(open.top())]) {open.pop();}
        curr = open.top(); open.pop();
        closed[TopHash(curr)] = 1;

        if(curr->m_lastAssigned == m_numWayPts) 
        {
            //return curr ;
            goalNode = curr;
            goal_expanded = 1;
            continue;
        } // Why it isn't m_numWaypts - 1

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
                    if (s->m_lastAssigned > 0) // how it can be 0? 
                        s->m_mid_level_path = MidSearch(s,s->m_WayPtAssignment[s->m_lastAssigned-1]); // else how the path will be assigned - will look into
                    // if (s->m_lastAssigned == 0)
                    //     cout << "ANAMOLY: " << *curr << endl;
                    open.push(s);
                }

                //cout << "\nTop Successor:" << *s << '\n';
            }
        }   

    }

    //cout << "TopSearch: " << goal_expanded << endl;
    if (goal_expanded)
    {
        TopPath.push_back(goalNode);
        while (goalNode->m_parent)
        {
            //cout << "\nTop Successor:" << *goalNode << '\n';
            goalNode = goalNode->m_parent;
            TopPath.push_back(goalNode);
        }

        reverse(TopPath.begin(), TopPath.end());

        /// Before returning the path let's print
        vector<GraphVertex*> printPath;
        printPath = TopPath;  
        for (int i = 0; i < printPath.size(); i++)
        {
            GraphVertex* topVertex = printPath[i];
            cout << "Top Vertex: " << *topVertex << endl;
            vector<GraphVertex*> MidPath = topVertex->m_mid_level_path;
            for (int m = 0; m < MidPath.size(); m++)
            {
                GraphVertex* midVertex = MidPath[m];
                cout << "Mid Vertex: " << *midVertex << endl;
                vector<GraphVertex*> LowPath = midVertex->m_low_level_path;
                cout << "Size of Low Path: " << LowPath.size() << endl;
                for (int l = 0; l < LowPath.size(); l++)
                {
                    GraphVertex* lowVertex = LowPath[l];
                    cout << "Low Vertex: " <<  *lowVertex << endl;
                }
            }
        }

        return TopPath;
    }

    return TopPath;
}


vector<GraphVertex*> MultiSurveillance::GetSuccessorsMid(GraphVertex* vertex, GraphVertex* end)
{
    vector<GraphVertex*> succ; GraphVertex* s;

    // [00,3] should not have successors; after reaching 3, do not generate any successors  
    if (vertex->m_lastVisited == m_numWayPts + 1)
    {
        return succ; 
    }

    for(int i = 0; i < m_numWayPts; i++)
    {
        if(vertex->m_WayPtVisitation[i] == 0 && end->m_WayPtVisitation[i] == 1) // going towards right here also 
        {
            vector<GraphVertex*> empty;
            s = new GraphVertex(vertex->m_WayPtVisitation, i+1, empty); // For mid level graph, omega becomes (i+1) - where is the robot currently at 
            s->m_WayPtVisitation[i] = 1;
            succ.push_back(s);
        }
    }

    if(vertex->m_lastVisited <= m_numWayPts)
    {
        vector<GraphVertex*> empty;
        succ.push_back(new GraphVertex(vertex->m_WayPtVisitation, m_numWayPts+1, empty));  // go directly to goal
    }

    return succ;
}

GraphVertex* MultiSurveillance::GetMidVertex(GraphVertex* goal, int robot)
{
    vector<bool> wayPtsToVisit(m_numWayPts, 0);
    for(int i = 0; i < m_numWayPts; i++)
        if(goal->m_WayPtAssignment[i] == robot)
            wayPtsToVisit[i] = 1;
    vector<GraphVertex*> empty; 
    return(new GraphVertex(wayPtsToVisit, m_numWayPts + 1, empty)); // goal for the mid-level search 
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


// Similar to LowSearch, MidSearch should return a path 
vector<GraphVertex*> MultiSurveillance::MidSearch(GraphVertex* goal, int robot)
{
    GraphVertex* end = GetMidVertex(goal, robot);
    priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority> open;
    unordered_map<int, bool> closed;
    unordered_map<int, GraphVertex*> vertices;

    double gVal_s; int hash_s;
    vector<GraphVertex*> succ;
    vector<GraphVertex*> empty;
    GraphVertex* curr = new GraphVertex(vector<bool>(m_numWayPts,0),0,empty);
    curr->m_parent = NULL; // Parent of start Node is set to NULL 
    curr->SetFValue(0,0);
    vertices[MidHash(curr)] = curr;
    open.push(curr);

    bool goal_expanded = 0;
    GraphVertex* goalNode;

    vector<GraphVertex*> MidPath;

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
        for(GraphVertex* s: succ)
        {
            hash_s = MidHash(s);
            if(!closed[hash_s])
            {
                vector<GraphVertex*> low_level_path = LowSearch(curr, s, robot); // I have to assign this low level path to Mid graph's node 
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

int MultiSurveillance::LowHash(GraphVertex* vertex)
{
    return ((vertex->m_Y)*(vertex->m_Xsz) + (vertex->m_X));
}

// Task 1: Change LowSearch to return a path // compiles but wrong answer 
vector<GraphVertex*> MultiSurveillance::LowSearch(GraphVertex* start, GraphVertex* goal, int robot) // should not just return the cost, but Path 
{
    //cout << "Start: " << *start << endl;
    //cout << "Goal: " << *goal << endl;
    // cout << "Printing robot starts at the start of every Low Search " << endl; 
    // for (int i = 0; i < m_starts.size(); i++)
    //     cout << m_starts[i].first << " " << m_starts[i].second << " robot " << robot << endl;  // robot could either be 1 or 2 ; why 0??

    pair<int,int> s, g;
    //cout << "start last Visited: " << start->m_lastVisited << endl; 
    s = (start->m_lastVisited == 0) ? m_starts[robot-1] : m_wayPts[start->m_lastVisited-1];
    //cout << "s.first: " << s.first << " m_starts[robot-1]: " << m_starts[robot-1].first << endl; 
    g = (goal->m_lastVisited == m_numWayPts+1) ? m_goals[robot-1] : m_wayPts[goal->m_lastVisited-1];

    // double eucDist = sqrt(pow(s.first-g.first,2) + pow(s.second-g.second,2));
    // return eucDist;

    priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority> open;
    unordered_map<int, bool> closed;
    unordered_map<int, GraphVertex*> vertices;

    double gVal_s; int hash_s;
    vector<GraphVertex*> succ;

    GraphVertex* curr = new GraphVertex(s, m_mapXsize, m_mapYsize);
    curr->m_parent = NULL;
    curr->SetFValue(0,0);
    //cout << "Start coordinates: " << s.first << " " << s.second << endl;
    vertices[LowHash(curr)] = curr;
    open.push(curr);
    GraphVertex* goalNode; // to store the goal for backtracking 
    bool goal_expanded = 0;
    while(!open.empty() && !goal_expanded)
    {
        while(closed[LowHash(open.top())]){ open.pop();}
        curr = open.top(); open.pop();
        closed[LowHash(curr)] = 1;

        if(curr->m_X == g.first && curr->m_Y == g.second)
        {
            goalNode = curr;
            goal_expanded = 1;
            continue; 
        }

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
                    s->m_parent = vertices[LowHash(curr)]; // Parent is getting updated here // but where is the parent assigned: here only, default equal 
                    open.push(s);
                }
            }
        }
    }

    vector<GraphVertex*> LowPath;

    if (goal_expanded)
    {
        // backtrack from variable goal // I can have a parent pointer
        LowPath.push_back(goalNode);
        // till start not reached 
        //while (goalNode->m_X != s.first || goalNode->m_Y != s.second) // both have to be 0 
        while (goalNode->m_parent)
        {
            goalNode = goalNode->m_parent;
            LowPath.push_back(goalNode);
        } 

        reverse(LowPath.begin(), LowPath.end());

        return LowPath;
    }

    return LowPath; // reversed the vector from start to goal 
    //return D_INF;
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
