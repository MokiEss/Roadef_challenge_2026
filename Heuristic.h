//
// Created by uha on 26/03/2026.
//

#ifndef ROADEF_CHALLENGE_2026_HEURISTIC_H
#define ROADEF_CHALLENGE_2026_HEURISTIC_H
#include "readerInstance.h"

struct CongestedArc {
    Arc arc;
    double saturation;

    bool operator<(const CongestedArc& other) const {
        return saturation > other.saturation;  // Sort descending
    }
};

class Heuristic {


public:
    Instance   &   inst;
    bool use_ftxui = false ;
    ResultBuilder & result_builder;
    Scenario & scenario;
    RoutingScheme rs;
    int i_max_decimal_places = 12;
    Heuristic(Instance   &   inst, bool use_ftxui, ResultBuilder & result_builder, Scenario & scenario ):inst(inst),use_ftxui(use_ftxui),
            result_builder(result_builder),scenario(scenario),rs(inst) {} ;
    bool RandomHeuristicRun();

    double computeMLU(int time_slot, const RoutingScheme& test_rs, int& most_congested_arc_id);

private:
    // New methods for congestion-aware routing
    std::vector<CongestedArc> getCongestedArcs(int time_slot, int top_k = 5);
    std::vector<Node> selectWaypointsAvoidingArcs(
        Node source, Node target,
        const std::vector<CongestedArc>& congested_arcs,
        int num_candidates = 3);
    double scoreWaypoint(Node waypoint, Node source, Node target,
                         const std::vector<CongestedArc>& congested_arcs);


};


#endif //ROADEF_CHALLENGE_2026_HEURISTIC_H