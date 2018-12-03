#ifndef MULTISURVEILLANCE_HPP
#define MULTISURVEILLANCE_HPP

#include "GraphVertex.hpp"
#include "ComparePriority.cpp"
#include <vector>

class MultiSurveillance
{
public:

    MultiSurveillance(int M, int N, vector<pair<int,int>> wayPts, 
        vector<pair<int,int>> starts, vector<pair<int,int>> goals,
        double* costmap, double collision_thresh, int xsz, int ysz);
    ~MultiSurveillance();

    double SearchCost(GraphVertex* reachedGoal);
    int TopHash(GraphVertex* vertex);
    void TopInitOpen(GraphVertex* current, unordered_map<int, GraphVertex*>* vertices, 
        priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority>* open);
    bool IsTopGoal(GraphVertex* vertex);
    vector<GraphVertex*> TopSuccessors(GraphVertex* vertex);
    double TopPathCost(GraphVertex* current, GraphVertex* successor);
    GraphVertex* TopSearch();

    GraphVertex* MidVertex(GraphVertex* goal, int robot);
    void MidInitOpen(GraphVertex* current, unordered_map<int, GraphVertex*>* vertices, 
        priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority>* open);
    int MidHash(GraphVertex* vertex);
    bool IsMidGoal(GraphVertex* vertex, GraphVertex* goal);
    vector<GraphVertex*> MidSuccessors(GraphVertex* vertex, GraphVertex* end);
    pair<double,GraphVertex*> MidPath(GraphVertex* current, GraphVertex* successor, int robot);
    GraphVertex* MidSearch(GraphVertex* goal, int robot);

    int LowHash(GraphVertex* vertex);
    void LowInitOpen(pair<int,int> start, GraphVertex* current, unordered_map<int, GraphVertex*>* vertices, 
        priority_queue<GraphVertex*, vector<GraphVertex*>, ComparePriority>* open);
    bool IsLowGoal(GraphVertex* vertex, pair<int,int> goal);
    vector<GraphVertex*> LowSuccessors(GraphVertex* vertex);
    double LowPathCost(GraphVertex* current, GraphVertex* successor);
    GraphVertex* LowSearch(GraphVertex* start, GraphVertex* goal, int robot);

    vector<pair<int,int>> BackTrackLowPlan(GraphVertex* midSearchPtr);
    pair<double***,int*> RunPlan();

    int m_numWayPts;
    int m_numRobots;
    vector<pair<int,int>> m_wayPts;
    vector<pair<int,int>> m_starts;
    vector<pair<int,int>> m_goals;

    double* m_costmap;
    double m_collThreshold;
    int m_mapXsize;
    int m_mapYsize;
    double m_plantime;
    double m_plancost;

};

#endif