#include "LowGraph.hpp"
#include "ComparePriorityLow.cpp"

#include <vector>
#include <iostream>
#include <cmath>
#include <queue>
#include <unordered_map>

#define D_INF numeric_limits<double>::infinity()

using namespace std;

LowGraph::LowGraph(double* map, int collision_thresh, int x_size, int y_size)
{ 
	m_map = map; 
    m_CollThresh = collision_thresh;
	m_Xsz = x_size;
	m_Ysz = y_size;
}

LowGraph::~LowGraph()
{
	//cout << "Low graph is destructed" << endl;
}

int LowGraph::LowHash(LowGraph::LowVertex* vertex)
{
    return ((vertex->m_Y)*(vertex->m_Xsz) + (vertex->m_X));
}

vector<LowGraph::LowVertex*> LowGraph::GetSuccessorsLow(LowGraph::LowVertex* vertex)
{
    vector<LowVertex*> succ;
    LowVertex* s; int n_X, n_Y;
    vector<int> dX{-1, -1, -1,  0,  0,  1, 1, 1};
    vector<int> dY{-1,  0,  1, -1,  1, -1, 0, 1};
    for(int i = 0; i < dX.size(); i++)
    {
        n_X = vertex->m_X + dX[i];
        n_Y = vertex->m_Y + dY[i];
        if(n_X < vertex->m_Xsz && n_X >= 0 && 
           n_Y < vertex->m_Ysz && n_Y >= 0 && m_map[n_Y*(vertex->m_Xsz)+n_X] < m_CollThresh) // collision-checking done here 
        {
            s = new LowVertex(pair<int,int>(n_X,n_Y), 
                           vertex->m_Xsz, vertex->m_Ysz);
            succ.push_back(s);
        }
    }
    return succ;
}

double LowGraph::LowSearchCost(pair<int, int>& s, pair<int, int>& g, int robot)
{
    priority_queue<LowVertex*, vector<LowVertex*>, ComparePriorityLow> open;
    unordered_map<int, bool> closed;
    unordered_map<int, LowVertex*> vertices;

    double gVal_s; int hash_s;
    vector<LowVertex*> succ;

    LowVertex* curr = new LowVertex(s, m_Xsz, m_Ysz);
    curr->m_parent = NULL;
    double hVal_s = sqrt(pow(g.first - curr->m_X,2) + pow(g.second - curr->m_Y,2));
    curr->SetFValue(0,hVal_s);
    //cout << "Start coordinates: " << s.first << " " << s.second << endl;
    vertices[LowHash(curr)] = curr;
    open.push(curr);
    LowVertex* goalNode; // to store the goal for backtracking 
    //bool goal_expanded = 0;
    while(!open.empty())
    {
        while(closed[LowHash(open.top())]){ open.pop();}
        curr = open.top(); open.pop();
        closed[LowHash(curr)] = 1;

        if(curr->m_X == g.first && curr->m_Y == g.second)
        {
            // goalNode = curr;
            // goal_expanded = 1;
            // continue; 
            return curr->m_gValue; 
        }

        succ = GetSuccessorsLow(curr);

        for(LowVertex* s: succ)
        {
            double cost =  sqrt(pow(s->m_X - curr->m_X,2) + pow(s->m_Y - curr->m_Y,2));
            gVal_s = curr->m_gValue + cost;
            hash_s = LowHash(s);
            if(!closed[hash_s])
            {
                hVal_s = sqrt(pow(g.first - s->m_X,2) + pow(g.second - s->m_Y,2));
                s->SetFValue(gVal_s,hVal_s);

                if(vertices[hash_s] == NULL) { vertices[hash_s] = s; }
                else { s = vertices[hash_s]; }
                
                if(gVal_s <= vertices[hash_s]->m_gValue)
                {
                    s->SetFValue(gVal_s,hVal_s);
                    s->m_parent = vertices[LowHash(curr)]; // Parent is getting updated here // but where is the parent assigned: here only, default equal 
                    open.push(s);
                }
            }
        }
    }

    return D_INF; 
}

vector<LowGraph::LowVertex*> LowGraph::LowSearch(pair<int,int>& s, pair<int,int>& g, int robot) // should not just return the cost, but Path 
{

    priority_queue<LowVertex*, vector<LowVertex*>, ComparePriorityLow> open;
    unordered_map<int, bool> closed;
    unordered_map<int, LowVertex*> vertices;

    double gVal_s; int hash_s;
    vector<LowVertex*> succ;

    LowVertex* curr = new LowVertex(s, m_Xsz, m_Ysz);
    curr->m_parent = NULL;
    double hVal_s = sqrt(pow(g.first - curr->m_X,2) + pow(g.second - curr->m_Y,2));
    curr->SetFValue(0,hVal_s);
    //cout << "Start coordinates: " << s.first << " " << s.second << endl;
    vertices[LowHash(curr)] = curr;
    open.push(curr);
    LowVertex* goalNode; // to store the goal for backtracking 
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

        for(LowVertex* s: succ)
        {
            double cost =  sqrt(pow(s->m_X - curr->m_X,2) + pow(s->m_Y - curr->m_Y,2));
            gVal_s = curr->m_gValue + cost;
            hash_s = LowHash(s);
            if(!closed[hash_s])
            {
                hVal_s = sqrt(pow(g.first - s->m_X,2) + pow(g.second - s->m_Y,2));
                s->SetFValue(gVal_s,hVal_s);

                if(vertices[hash_s] == NULL) { vertices[hash_s] = s; }
                else { s = vertices[hash_s]; }
                
                if(gVal_s <= vertices[hash_s]->m_gValue)
                {
                    s->SetFValue(gVal_s,hVal_s);
                    s->m_parent = vertices[LowHash(curr)]; // Parent is getting updated here // but where is the parent assigned: here only, default equal 
                    open.push(s);
                }
            }
        }
    }

    vector<LowVertex*> LowPath;

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

// int main()
// {
// 	int M = 2;
// 	vector<vector<int>> map(8, vector<int>(8,0));
// 	LowGraph::LowGraph *GL = new LowGraph(map);
// 	GL->~LowGraph();
// 	return 0;
// }