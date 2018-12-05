
#include "TopGraph.hpp"
#include "ComparePriorityTop.cpp"

#include <vector>
#include <iostream>
#include <cmath>
#include <queue>

#define D_INF numeric_limits<double>::infinity()

using namespace std;

TopGraph::TopGraph(int numWayPts, int numRobots, vector<pair<int,int>> wayPts, vector<pair<int,int>> starts, vector<pair<int,int>> goals, 
    double* map, int collision_thresh,
    int x_size, int y_size)
{
	m_numWayPts = numWayPts; 
	m_numRobots = numRobots;
	m_starts = starts;
	m_goals = goals;
	m_wayPts = wayPts;
	m_map = map;
    m_Xsz = x_size;
    m_Ysz = y_size; 	
    m_CollThresh = collision_thresh;
	//cout << "Top Graph is constructed" << endl;
}

TopGraph::~TopGraph()
{
	//cout << "Top graph is destructed" << endl;
}

int TopGraph::TopHash(TopVertex* vertex)
{
	vector<int> vec = vertex->m_WayPtAssignment;    
    int seed = vec.size();
    for(auto& i : vec) 
    {
        seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
}

vector<TopGraph::TopVertex*> TopGraph::GetSuccessorsTop(TopVertex* vertex, unordered_map<int, TopVertex*> vertices)
{
	vector<TopVertex*> succ;
    TopVertex *s, *parent = vertices[TopHash(vertex)]; // parent is simply the vertex 
    int nextAssign = vertex->m_lastAssigned + 1; // going towards right 

    if (nextAssign <= m_numWayPts)
    {
        for (int i = 0; i < m_numRobots; i++)
        {
            vector<MidGraph::MidVertex*> empty;
            s = new TopVertex(vertex->m_WayPtAssignment, nextAssign, vertex->m_WayPtAssignmentCosts, parent, empty);
            s->m_WayPtAssignment[nextAssign-1] = i+1; // assigned the robot id at index nextAssign-1

            MidGraph::MidGraph *GM = new MidGraph(m_numWayPts, m_starts, m_goals, m_wayPts, m_map, m_CollThresh, m_Xsz, m_Ysz);
            //vector<MidGraph::MidVertex*> mid_level_path = GM->MidSearch(s->m_WayPtAssignment,i+1);
            double MidCost = GM->MidSearchCost(s->m_WayPtAssignment,i+1);
            GM->~MidGraph();
            //double MidCost = mid_level_path.back()->m_gValue;

            s->m_WayPtAssignmentCosts[i] = MidCost;
            //s->m_WayPtAssignmentCosts[i] = MidSearch(s,i+1);
            //cout << " Mid search: " << MidSearch(s,i+1) << endl;
            succ.push_back(s);
        }
    }

    return succ;
}

double TopGraph::TopPathCost(TopGraph::TopVertex* current, TopGraph::TopVertex* successor) // MidSearch returns the cost of edge  - Handles the edge case when robot has to plan from start to goal with no waypoint visitation  
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

        	MidGraph::MidGraph *GM = new MidGraph(m_numWayPts, m_starts, m_goals, m_wayPts, m_map, m_CollThresh, m_Xsz , m_Ysz);
            //vector<MidGraph::MidVertex*> mid_level_path = GM->MidSearch(successor->m_WayPtAssignment, i+1);
            double MidSearchCost = GM->MidSearchCost(successor->m_WayPtAssignment,i+1);
            GM->~MidGraph();

            //double MidSearchCost = mid_level_path.back()->m_gValue;
            //gVal_s += MidSearch(successor, i+1);
            gVal_s += MidSearchCost;
        }
    }

    return gVal_s;
}

vector<vector<pair<int, int>>> TopGraph::TopSearch()
{

    /// 3D vector: NumRobots x pathlen x 2 
    vector<vector<pair<int,int>>> paths_of_all_robots(m_numRobots); // size equal to NumRobots : for index access 

    priority_queue<TopVertex*, vector<TopVertex*>, ComparePriorityTop> open;
    unordered_map<int, bool> closed;
    unordered_map<int, TopVertex*> vertices;

    double gVal_s; int robot_s, hash_s; // what does robot_s do?
    vector<TopVertex*> succ;

    vector<MidGraph::MidVertex*> empty; 
    TopVertex* curr = new TopVertex(vector<int>(m_numWayPts,0), 0, vector<double>(m_numRobots,0), NULL, empty);
    curr->m_parent = NULL;
    curr->SetFValue(0,0);
    vertices[TopHash(curr)] = curr;
    open.push(curr);

    vector<TopVertex*> TopPath;
    bool goal_expanded = 0;
    TopVertex* goalNode; 

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

        for(TopVertex* s: succ)
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
                    if (s->m_lastAssigned > 0) 
                    {
                    	MidGraph::MidGraph *GM = new MidGraph(m_numWayPts, m_starts, m_goals, m_wayPts, m_map, m_CollThresh, m_Xsz, m_Ysz);
                        //s->m_mid_level_path = GM->MidSearch(s->m_WayPtAssignment,s->m_WayPtAssignment[s->m_lastAssigned-1]); // else how the path will be assigned - will look into
                        GM->~MidGraph();
                    }
                    // if (s->m_lastAssigned == 0)
                    //     cout << "ANAMOLY: " << *curr << endl;
                    open.push(s);
                }

                //cout << "\nTop Successor:" << *s << '\n';
            }
        }   

    }

    //cout << "TopSearch: " << goal_expanded << endl;
    int robot_id; 
    if (goal_expanded)
    {
        double path_cost = 0.0;

        cout << "Path Cost: " << goalNode->m_gValue << endl; 

        TopPath.push_back(goalNode);
        while (goalNode->m_parent)
        {
            //cout << "\nTop Successor:" << *goalNode << '\n';
            goalNode = goalNode->m_parent;
            TopPath.push_back(goalNode);
        }

        reverse(TopPath.begin(), TopPath.end());

        /// Before returning the path let's print

        vector<TopVertex*> printPath;
        printPath = TopPath;  

        // Extract Oprimal allocation 
        TopVertex* optimal_allocation = printPath.back();

        // print optimal allocation 
        //cout << *optimal_allocation << endl;

        // Generate paths again and print 
        for (int r = 0; r < m_numRobots; r++)
        {
        	MidGraph::MidGraph *GM = new MidGraph(m_numWayPts, m_starts, m_goals, m_wayPts, m_map, m_CollThresh, m_Xsz, m_Ysz);
            vector<MidGraph::MidVertex*> robot_mid_path = GM->MidSearch(optimal_allocation->m_WayPtAssignment, r+1);
            for (int m = 0; m < robot_mid_path.size(); m++)
            {
                MidGraph::MidVertex* midVertex = robot_mid_path[m];
                vector<LowGraph::LowVertex*> LowPath = midVertex->m_low_level_path;
                for (int l = 0; l < LowPath.size(); l++)           
                {
                    LowGraph::LowVertex* lowVertex = LowPath[l];
                    //cout << "Low Vertex: " <<  *lowVertex << endl;
                    paths_of_all_robots[r].push_back({lowVertex->m_X, lowVertex->m_Y}); // pair of (x,y) pushed to the path of corresponding robot 
                }            
            }
            GM->~MidGraph();
        }

    }

    return paths_of_all_robots;
}

double SqrEucDist(pair<int,int> s, pair<int,int> g)
{
    return(pow(s.first-g.first,2) + pow(s.second-g.second,2));
}

vector<vector<pair<int, int>>> TopGraph::GreedyAssignment()
{
    vector<MidGraph::MidVertex*> empty; 
    TopVertex* curr = new TopVertex(vector<int>(m_numWayPts,0), 0, vector<double>(m_numRobots,0), NULL, empty);

    vector<pair<int,int>> robPoses = m_starts;
    double dist, minDist; int robAssign, wayptAssign;
    bool assigned = true;

    while(assigned)
    {
        assigned = false;
        minDist = D_INF;
         for(int j = 0; j < m_numWayPts; j++)
            if(curr->m_WayPtAssignment[j] == 0)
            {
                assigned = true;
                for(int i = 0; i < m_numRobots; i++)
                {
                    dist = SqrEucDist(robPoses[i], m_wayPts[j]);
                    //dist = SearchCost(LowSearchGreedy(robPoses[i], m_wayPts[j]));
                    if(dist < minDist)
                    {
                        minDist = dist;
                        robAssign = i;
                        wayptAssign = j;
                    }
                }
            }
        if(assigned)
        { 
            curr->m_WayPtAssignment[wayptAssign] = robAssign+1; 
            robPoses[robAssign] = m_wayPts[wayptAssign];
        }
    }

   /// 3D vector: NumRobots x pathlen x 2 
    vector<vector<pair<int,int>>> paths_of_all_robots(m_numRobots); // size equal to NumRobots : for index access 

    priority_queue<TopVertex*, vector<TopVertex*>, ComparePriorityTop> open;
    unordered_map<int, bool> closed;
    unordered_map<int, TopVertex*> vertices;

    double gVal_s; int robot_s, hash_s; // what does robot_s do?
    vector<TopVertex*> succ;

    //vector<MidGraph::MidVertex*> empty; 
    //TopVertex* curr = new TopVertex(vector<int>(m_numWayPts,0), 0, vector<double>(m_numRobots,0), NULL, empty);
    curr->m_parent = NULL;
    curr->SetFValue(0,0);
    vertices[TopHash(curr)] = curr;
    open.push(curr);

    vector<TopVertex*> TopPath;
    bool goal_expanded = 0;
    TopVertex* goalNode; 

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

        for(TopVertex* s: succ)
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
                    if (s->m_lastAssigned > 0) 
                    {
                        MidGraph::MidGraph *GM = new MidGraph(m_numWayPts, m_starts, m_goals, m_wayPts, m_map, m_CollThresh, m_Xsz, m_Ysz);
                        //s->m_mid_level_path = GM->MidSearch(s->m_WayPtAssignment,s->m_WayPtAssignment[s->m_lastAssigned-1]); // else how the path will be assigned - will look into
                        GM->~MidGraph();
                    }
                    // if (s->m_lastAssigned == 0)
                    //     cout << "ANAMOLY: " << *curr << endl;
                    open.push(s);
                }

                //cout << "\nTop Successor:" << *s << '\n';
            }
        }   

    }

    //cout << "TopSearch: " << goal_expanded << endl;
    int robot_id; 
    if (goal_expanded)
    {
        double path_cost = 0.0;

        cout << "Path Cost: " << goalNode->m_gValue << endl; 

        TopPath.push_back(goalNode);
        while (goalNode->m_parent)
        {
            //cout << "\nTop Successor:" << *goalNode << '\n';
            goalNode = goalNode->m_parent;
            TopPath.push_back(goalNode);
        }

        reverse(TopPath.begin(), TopPath.end());

        /// Before returning the path let's print

        vector<TopVertex*> printPath;
        printPath = TopPath;  

        // Extract Oprimal allocation 
        TopVertex* optimal_allocation = printPath.back();

        // print optimal allocation 
        //cout << *optimal_allocation << endl;

        // Generate paths again and print 
        for (int r = 0; r < m_numRobots; r++)
        {
            MidGraph::MidGraph *GM = new MidGraph(m_numWayPts, m_starts, m_goals, m_wayPts, m_map, m_CollThresh, m_Xsz, m_Ysz);
            vector<MidGraph::MidVertex*> robot_mid_path = GM->MidSearch(optimal_allocation->m_WayPtAssignment, r+1);
            for (int m = 0; m < robot_mid_path.size(); m++)
            {
                MidGraph::MidVertex* midVertex = robot_mid_path[m];
                vector<LowGraph::LowVertex*> LowPath = midVertex->m_low_level_path;
                for (int l = 0; l < LowPath.size(); l++)           
                {
                    LowGraph::LowVertex* lowVertex = LowPath[l];
                    //cout << "Low Vertex: " <<  *lowVertex << endl;
                    paths_of_all_robots[r].push_back({lowVertex->m_X, lowVertex->m_Y}); // pair of (x,y) pushed to the path of corresponding robot 
                }            
            }
            GM->~MidGraph();
        }

    }

    return paths_of_all_robots;
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

// 	TopGraph::TopGraph *GT = new TopGraph(M, N, wayPts, starts, goals, map);
// 	vector<vector<pair<int,int>>> paths_of_all_robots = GT->TopSearch();

// 	GT->~TopGraph();

// 	for (int r = 0; r < N; r++)
//     {
//         cout << r << endl;
//         for (int j = 0; j < paths_of_all_robots[r].size(); j++)
//         {
//             cout << paths_of_all_robots[r][j].first << " " << paths_of_all_robots[r][j].second << endl;
//         }
//     }

// 	return 0;
// }