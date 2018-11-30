#ifndef MULTISURVEILLANCE_HPP
#define MULTISURVEILLANCE_HPP

#include "GraphVertex.hpp"
#include <vector>

class MultiSurveillance
{
public:

    MultiSurveillance(int M, int N, vector<pair<int,int>> wayPts, 
        vector<pair<int,int>> starts, vector<pair<int,int>> goals,
        double* costmap, double collision_thresh, int xsz, int ysz);
    ~MultiSurveillance();

    int TopHash(GraphVertex* vertex);
    vector<GraphVertex*> TopSuccessors(GraphVertex* vertex);
    double TopPathCost(GraphVertex* current, GraphVertex* successor);
    GraphVertex* TopSearch();

    GraphVertex* MidVertex(GraphVertex* goal, int robot);
    int MidHash(GraphVertex* vertex);
    vector<GraphVertex*> MidSuccessors(GraphVertex* vertex, GraphVertex* end);
    double MidPathCost(GraphVertex* current, GraphVertex* successor, int robot);
    double MidSearch(GraphVertex* goal, int robot);

    int LowHash(GraphVertex* vertex);
    vector<GraphVertex*> LowSuccessors(GraphVertex* vertex);
    double LowPathCost(GraphVertex* current, GraphVertex* successor);
    double LowSearch(GraphVertex* start, GraphVertex* goal, int robot);

    int m_numWayPts;
    int m_numRobots;
    vector<pair<int,int>> m_wayPts;
    vector<pair<int,int>> m_starts;
    vector<pair<int,int>> m_goals;

    double* m_costmap;
    double m_collThreshold;
    int m_mapXsize;
    int m_mapYsize;

};

#endif