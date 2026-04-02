//
// Created by uha on 26/03/2026.
//

#include "Heuristic.h"

double Heuristic::computeMLU(int time_slot, const RoutingScheme& test_rs, int& most_congested_arc_id) {
    SegmentRouting sr(inst.network, inst.metrics);
    const int i_num_routed = sr.run(inst.demand_graph, inst.dvms[time_slot], test_rs.getSrPathsAt(time_slot));

    if (i_num_routed != inst.demand_graph.arcNum()) {
        most_congested_arc_id = -1;
        return std::numeric_limits<double>::infinity(); // Invalid routing
    }

    double f_mlu = 0.;
    most_congested_arc_id = -1;

    for (ArcIt arc(inst.network); arc != nt::INVALID; ++arc) {
        const double f_sat = sr.saturation(arc, inst.capacities[arc]);
        if (f_sat > f_mlu) {
            f_mlu = f_sat;
            most_congested_arc_id = inst.network.id(arc);
        }
    }

    return f_mlu;
}

vector<Node> getWayPointsCandidates(const Instance& inst, Node source, Node target) {
    vector<Node> candidates;
    for (NodeIt node(inst.network); node != nt::INVALID; ++node) {
        if (node != source && node != target) {
            candidates.push_back(node);
        }
    }

    return candidates;
}


bool Heuristic::RandomHeuristicRun() {
    result_builder.setValid(true);
    int i_total_segments = 0;      // Accumulate total number of segments across all SR paths
    int i_num_srpaths = 0;   // Count only srpaths with more than one segment
    int total_cost = 0 ;
    // ===== LOOP TO ADD WAYPOINTS =====

    for (int t = 0; t < inst.i_num_time_slots; ++t) {
        std::cout << "=== Time Slot " << t << " ===" << std::endl;
        // Iterate through all demands
             for (int i = 0; i < inst.demand_graph.arcNum(); i++) {
                 if ((t==0) || (t>0 && total_cost < scenario.budget[t])) {
                    int i_num_segments = 0;
                    DemandArc demand_arc = inst.demand_graph.arcFromId(i);
                    Node source = inst.demand_graph.source(demand_arc);
                    Node target = inst.demand_graph.target(demand_arc);
                    // Get the SR path for this time slot and demand
                    SrPathBit & path = rs.getSrPath(t, demand_arc);
                    // Initialize with direct path
                    if (t==0) {
                        path.init(inst.network, 3);  // 10 = estimated capacity for waypoints
                        path.addSegment(source);
                        path.finalize(target);
                    }
                    else {
                        path.copyFrom(rs.getSrPath(t-1, demand_arc));
                    }
                    int worst_arc;
                    double best_mlu = computeMLU(t, rs, worst_arc);
                    SrPathBit best_path;
                     best_path.copyFrom(path);

                    // Try several random waypoints
                    const int num_attempts = 5;  // Adjust based on time constraints
                    vector<Node> WayPointsCandidates;
                    WayPointsCandidates = getWayPointsCandidates(inst, source, target);
                    std::shuffle(WayPointsCandidates.begin(), WayPointsCandidates.end(), std::mt19937{std::random_device{}()});

                    for (int attempt = 0; attempt < num_attempts; ++attempt) {
                            Node p_wp = WayPointsCandidates[attempt];
                            if (p_wp != source && p_wp != target) {
                                // Temporarily set path with waypoint
                                path.init(inst.network, 3);
                                path.addSegment(source);
                                path.addSegment(p_wp, source);
                                path.finalize(target);
                                int worst_arc;
                                double current_mlu = computeMLU(t, rs, worst_arc);
                                if (current_mlu < best_mlu && t==0) {
                                    best_mlu = current_mlu;
                                    best_path.copyFrom(path);

                                    cout << "It is improved at t " << t << " for demand " <<
                                        inst.demand_graph.id(demand_arc) <<" and for arc " << worst_arc <<
                                            " and MLU is " << best_mlu << endl  ;
                                }
                                if (current_mlu < best_mlu && t>0) {
                                    if (total_cost + dist(rs.getSrPath(t - 1, demand_arc), path) <= scenario.budget[t]) {
                                        best_mlu = current_mlu;
                                        best_path.copyFrom(path);
                                        cout << "It is improved at t " << t << " for demand " <<
                                            inst.demand_graph.id(demand_arc) <<" and for arc " << worst_arc <<
                                                " and MLU is " << best_mlu << endl  ;
                                        total_cost += dist(rs.getSrPath(t - 1, demand_arc), path);
                                        cout << " and total cost for demand : " << inst.demand_graph.id(demand_arc) << " is " << total_cost << endl ;
                                    }
                                }

                        }
                    }
                    // copy the best path into the main solution
                    path.copyFrom(best_path);
                    i_num_segments = path.segmentNum() - 1;
                    i_total_segments += i_num_segments;      // Accumulate total number of segments across all SR paths
                    i_num_srpaths += (i_num_segments > 1);   // Count only srpaths with more than one segment
                }
                 else {
                     cout << "Budget exceeded or reached at time slot " << t << " with total cost " << total_cost << " and budget " << scenario.budget[t] << endl;
                     break;  // Stop if budget is exceeded

                 }
        }


    }
    // ===== END OF LOOP =====
    if (!simulateSegmentRouting(inst, scenario, rs, result_builder)) {
        result_builder.setValid(false);
        result_builder.display(i_max_decimal_places);
        return false;
    }
    cout << "Results:" << endl;
    result_builder._i_total_segments = i_total_segments;
    result_builder._i_total_srpaths = i_num_srpaths;
    result_builder._i_total_cost = total_cost;
    std::sort(
    result_builder._sat_values.begin(),
    result_builder._sat_values.end(),
    [](const auto& a, const auto& b) {
        return a.third > b.third; // Sort descending by saturation
    }
    );
  /*  for (int s = 0 ; s < result_builder._sat_values.size() ; s++) {
        auto triplet = result_builder._sat_values[s];
        int source = inst.network.id(inst.network.source(triplet.second));
        int target = inst.network.id(inst.network.target(triplet.second));
        cout << "Time slot: " << triplet.first << ", Arc: (" << inst.network.id(triplet.second)
        << " " << source << " , "<< target << "), Saturation: " << triplet.third << endl;
    }*/

    return true;
}


