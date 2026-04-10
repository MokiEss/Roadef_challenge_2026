//
// Created by uha on 26/03/2026.
//

#ifndef ROADEF_CHALLENGE_2026_HEURISTIC_H
#define ROADEF_CHALLENGE_2026_HEURISTIC_H
#include "readerInstance.h"
using DemandArray = nt::TrivialDynamicArray<DemandArc>;
// In Heuristic.h

struct NetworkPrecompute {
    std::vector<std::vector<double>>          dist_matrix;
    std::vector<std::unordered_set<int>>   neighbors;
};

struct CongestedArc {
    Arc arc;
    double saturation;

    bool operator<(const CongestedArc& other) const {
        return saturation > other.saturation;  // Sort descending
    }
};


// Remove flow of one demand from the network state
// Returns new arc loads, saturations, and MLU — without rerouting other demands


class Heuristic {


public:
    Instance   &   inst;
    bool use_ftxui = false ;
    ResultBuilder & result_builder;
    Scenario & scenario;
    RoutingScheme rs;
    int i_max_decimal_places = 12;
    SegmentRouting & sr ;
    Heuristic(Instance   &   inst, bool use_ftxui, ResultBuilder & result_builder, Scenario & scenario, SegmentRouting & sr):inst(inst),use_ftxui(use_ftxui),
            result_builder(result_builder),scenario(scenario),rs(inst), sr(sr) {

    } ;

    bool RandomHeuristicRun();
    bool ArcJumpHeuristicRun();
    bool newHeuristicRun();
    double computeMLU(int time_slot, const RoutingScheme& test_rs, int& most_congested_arc_id);
    double computeMLU(SegmentRouting & sr, int time_slot,  int& most_congested_arc_id,
    DemandArc demand_arc,const SrPathBit& old_path, const SrPathBit& path, Digraph::ArcMap<DemandArray> & dpa);



    void computeAllPairsShortestPaths(NetworkPrecompute& precomp) ;
    Node selectGeometricWaypoint(Node s, Node d, Arc worst_arc, const NetworkPrecompute& precomp) const;



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