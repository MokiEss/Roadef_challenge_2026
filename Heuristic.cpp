//
// Created by uha on 26/03/2026.
//

#include "Heuristic.h"

double Heuristic::computeMLU(int time_slot, const RoutingScheme& test_rs) {
    SegmentRouting sr(inst.network, inst.metrics);
    const int i_num_routed = sr.run(inst.demand_graph, inst.dvms[time_slot], test_rs.getSrPathsAt(time_slot));

    if (i_num_routed != inst.demand_graph.arcNum()) {
        return std::numeric_limits<double>::infinity(); // Invalid routing
    }

    double f_mlu = 0.;
    for (ArcIt arc(inst.network); arc != nt::INVALID; ++arc) {
        const double f_sat = sr.saturation(arc, inst.capacities[arc]);
        f_mlu = nt::max(f_mlu, f_sat);
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

    // ===== LOOP TO ADD WAYPOINTS =====
    for (int t = 0; t < inst.i_num_time_slots; ++t) {
        std::cout << "=== Time Slot " << t << " ===" << std::endl;

        // Iterate through all demands
        for (int i = 0; i < inst.demand_graph.arcNum(); i++) {
            DemandArc demand_arc = inst.demand_graph.arcFromId(i);
            Node source = inst.demand_graph.source(demand_arc);
            Node target = inst.demand_graph.target(demand_arc);

            // Get the SR path for this time slot and demand
            SrPathBit & path = rs.getSrPath(t, demand_arc);

            // Initialize with direct path
            path.init(inst.network, 3);  // 10 = estimated capacity for waypoints
            path.addSegment(source);
            path.finalize(target);

            double best_mlu = computeMLU(t, rs);
            SrPathBit best_path;
            best_path.copyFrom(path);
            // Try several random waypoints
            const int num_attempts = 5;  // Adjust based on time constraints
            vector<Node> WayPointsCandidates;
            WayPointsCandidates = getWayPointsCandidates(inst, source, target);
            std::shuffle(WayPointsCandidates.begin(), WayPointsCandidates.end(), std::mt19937{std::random_device{}()});
            for (int attempt = 0; attempt < num_attempts; ++attempt) {
                if (genRandomDouble(1) < 1) {
                    Node p_wp = WayPointsCandidates[attempt];
                    if (p_wp != source && p_wp != target) {
                        // Temporarily set path with waypoint
                        path.init(inst.network, 3);
                        path.addSegment(source);
                        path.addSegment(p_wp, source);
                        path.finalize(target);

                        double current_mlu = computeMLU(t, rs);
                        if (current_mlu < best_mlu) {
                            best_mlu = current_mlu;
                            best_path.copyFrom(path);
                            cout << "It is improved at t " << t << " and MLU is " << best_mlu << endl ;
                        }
                    }
                }
            }
            // copy the best path into the main solution
            path.copyFrom(best_path);
            cout << "Path segments: " << path.segmentNum()-1 << endl;
        }
    }
    // ===== END OF LOOP =====
    if (!simulateSegmentRouting(inst, scenario, rs, result_builder)) {
        result_builder.setValid(false);
        result_builder.display(i_max_decimal_places);
        return false;
    }
    cout << "Results:" << endl;
    for (int i = 0; i < result_builder._mlu_values.size(); i++) {
        cout << "t=" << i << " mlu=" << result_builder._mlu_values[i] << endl;
    }
    result_builder.display(i_max_decimal_places);
    return true;
}


