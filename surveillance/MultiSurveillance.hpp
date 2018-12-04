#ifndef MULTISURVEILLANCE_HPP
#define MULTISURVEILLANCE_HPP

#include "GraphVertex.hpp"
#include "ComparePriority.cpp"
#include <memory>
#include <vector>

class MultiSurveillance
{
public:

    MultiSurveillance(int M, int N, vector<pair<int,int>> wayPts, 
        vector<pair<int,int>> starts, vector<pair<int,int>> goals,
        double* costmap, double collision_thresh, int xsz, int ysz);
    ~MultiSurveillance();
    
    typedef std::shared_ptr<GraphVertex> GraphVertexPtr_t;

    double SearchCost(GraphVertexPtr_t reachedGoal);
    int TopHash(GraphVertexPtr_t vertex);
    void TopInitOpen(GraphVertexPtr_t current, unordered_map<int, GraphVertexPtr_t>* vertices, 
        priority_queue<GraphVertexPtr_t, vector<GraphVertexPtr_t>, ComparePriority>* open);
    bool IsTopGoal(GraphVertexPtr_t vertex);
    vector<GraphVertexPtr_t> TopSuccessors(GraphVertexPtr_t vertex);
    double TopPathCost(GraphVertexPtr_t current, GraphVertexPtr_t successor);
    GraphVertexPtr_t TopSearch();

    GraphVertexPtr_t MidVertex(GraphVertexPtr_t goal, int robot);
    void MidInitOpen(GraphVertexPtr_t current, unordered_map<int, GraphVertexPtr_t>* vertices, 
        priority_queue<GraphVertexPtr_t, vector<GraphVertexPtr_t>, ComparePriority>* open);
    int MidHash(GraphVertexPtr_t vertex);
    bool IsMidGoal(GraphVertexPtr_t vertex, GraphVertexPtr_t goal);
    vector<GraphVertexPtr_t> MidSuccessors(GraphVertexPtr_t vertex, GraphVertexPtr_t end);
    pair<double,GraphVertexPtr_t> MidPath(GraphVertexPtr_t current, GraphVertexPtr_t successor, int robot);
    GraphVertexPtr_t MidSearch(GraphVertexPtr_t goal, int robot);

    int LowHash(GraphVertexPtr_t vertex);
    void LowInitOpen(pair<int,int> start, GraphVertexPtr_t current, unordered_map<int, GraphVertexPtr_t>* vertices, 
        priority_queue<GraphVertexPtr_t, vector<GraphVertexPtr_t>, ComparePriority>* open);
    bool IsLowGoal(GraphVertexPtr_t vertex, pair<int,int> goal);
    vector<GraphVertexPtr_t> LowSuccessors(GraphVertexPtr_t vertex);
    double LowPathCost(GraphVertexPtr_t current, GraphVertexPtr_t successor);
    GraphVertexPtr_t LowSearch(GraphVertexPtr_t start, GraphVertexPtr_t goal, int robot);

    vector<pair<int,int>> BackTrackLowPlan(GraphVertexPtr_t midSearchPtr);
    pair<double***,int*> RunPlan(int assignmentType);
    GraphVertexPtr_t RandomAssignment();
    GraphVertexPtr_t GreedyAssignment();
    double SqrEucDist(pair<int,int> s, pair<int,int> g);

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