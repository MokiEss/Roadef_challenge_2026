//
// Created by uha on 26/03/2026.
//

#include "Heuristic.h"

bool Heuristic::run() {
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
            SrPathBit& path = rs.getSrPath(t, demand_arc);

            // ✓ IMPORTANT: Initialize the path with the network
            path.init(inst.network, 3);  // 10 = estimated capacity for waypoints

            // Now add segments
            path.addSegment(source);
            if (genRandomDouble(1.0) < 0.2) {


                // Generate random intermediate node
                int randomNode = genRandomInt(inst.network.nodeNum());
                cout << "random node: " << randomNode << " (max: " << inst.network.nodeNum() << ")" << endl;

                Node p_wp = inst.network.nodeFromId(randomNode);

                // Add waypoint with proper signature
                if (p_wp != source && p_wp != target) {
                    path.addSegment(p_wp, source);
                    cout << "Added waypoint: " << inst.names[p_wp] << endl;
                }
            }

            // Finalize with target
            path.finalize(target);
            cout << "Path segments: " << path.segmentNum() << endl;
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

/*
bool Heuristic::run() {

    result_builder.setValid(true);
    // ===== LOOP TO ADD WAYPOINTS =====
    for (int t = 0; t < inst.i_num_time_slots; ++t) {
        std::cout << "=== Time Slot " << t << " ===" << std::endl;

        // Iterate through all demands
        for (int i = 0 ; i < inst.demand_graph.arcNum(); i++) {
            DemandArc demand_arc = inst.demand_graph.arcFromId(i);
            // Get source and target of this demand
            Node source = inst.demand_graph.source(demand_arc);

            // Get the SR path for this time slot and demand
            SrPathBit& path = rs.getSrPath(t, demand_arc);

            // Initialize path with source
            path.addSegment(source);
            int randomNode = genRandomInt(inst.network.nodeNum());
            cout << "random node " << randomNode <<"inst.network.nodeNum() "<< inst.network.nodeNum() <<  endl ;
            Node p_wp = inst.network.nodeFromId(randomNode);
            cout << "im here " << endl ;

            try {
                path.addSegment(p_wp, source);
            }
            catch (const std::exception& e) {
                LOG_F(ERROR, "Error adding segment: %s", e.what());
                return false;
            }

            source = p_wp;
            path.finalize(inst.demand_graph.target(demand_arc));
            cout << "im here 3" << endl ;


        }
    }
    // ===== END OF LOOP =====
    if (!simulateSegmentRouting(inst, scenario, rs, result_builder)) {
        result_builder.setValid(false);
        result_builder.display(i_max_decimal_places);
        cout << "im here" << endl ;
        return false;
    }
    cout << result_builder._mlu_values.size() << endl;
    for (int i = 0 ; i < result_builder._mlu_values.size() ; i++) {
        cout << "maximum value " << result_builder._mlu_values[i] << endl;
    }

    cout << "--------------------------" << endl ;
    result_builder.display(i_max_decimal_places);
    return true;
}
*/
