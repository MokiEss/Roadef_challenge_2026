//
// Created by uha on 26/03/2026.
//

#include "Heuristic.h"



double Heuristic::computeMLU(SegmentRouting & sr, int time_slot,  int& most_congested_arc_id,
    DemandArc demand_arc,const SrPathBit& old_path, const SrPathBit& path, Digraph::ArcMap<DemandArray> & dpa, bool update ) {
    double flow = inst.dvms[time_slot][demand_arc];
    // Remove old flow, add new flow (no dpa involved)

    sr.routeFlow(old_path, -flow,demand_arc,dpa);
    sr.routeFlow(path, flow,demand_arc,dpa);

    auto most = sr.mostLoadedArc(inst.capacities);
    double mlu = most.second;
    most_congested_arc_id = (most.first == nt::INVALID) ? -1 : Digraph::id(most.first);
    // Undo: restore to original state
    if (update == false) {
        sr.routeFlow(path, -flow,demand_arc,dpa);
        sr.routeFlow(old_path, flow,demand_arc,dpa);
    }
    return mlu ;
}



Node Heuristic::selectGeometricWaypoint(Node s, Node d, Arc worst_arc, const NetworkPrecompute& precomp) const {
    const Node u = inst.network.source(worst_arc);
    const Node v = inst.network.target(worst_arc);

    const int sid = inst.network.id(s);
    const int did = inst.network.id(d);
    const int uid = inst.network.id(u);
    const int vid = inst.network.id(v);

    const double d_sd = precomp.dist_matrix[sid][did];
    if (!std::isfinite(d_sd) || d_sd <= 0.0) return nt::INVALID;

    Node best_w = nt::INVALID;
    double best_score = -std::numeric_limits<double>::infinity();

    for (NodeIt w(inst.network); w != nt::INVALID; ++w) {
        if (w == s || w == d || w == u || w == v) continue;

        const int wid = inst.network.id(w);
        const double d_sw = precomp.dist_matrix[sid][wid];
        const double d_wd = precomp.dist_matrix[wid][did];
        if (!std::isfinite(d_sw) || !std::isfinite(d_wd)) continue;

        const double detour = (d_sw + d_wd) / d_sd;
        if (detour > 1.35) continue;

        const double sep = std::min(precomp.dist_matrix[wid][uid], precomp.dist_matrix[wid][vid]);
        if (!std::isfinite(sep)) continue;

        const double balance = std::abs(d_sw - d_wd) / (d_sw + d_wd + 1e-9);
        const double score = 2.0 * sep - 1.2 * detour - 0.5 * balance;

        if (score > best_score) {
            best_score = score;
            best_w = w;
        }
    }

    return best_w;
}



double Heuristic::computeMLU(int time_slot, const RoutingScheme& test_rs, int& most_congested_arc_id) {
    sr.clear();
    InterventionGuard intervention_guard(inst.network, inst.metrics, scenario.interventions[time_slot]);
    const int i_num_routed = sr.run(inst.demand_graph, inst.dvms[time_slot], test_rs.getSrPathsAt(time_slot));

    if (i_num_routed != inst.demand_graph.arcNum()) {
        most_congested_arc_id = -1;
        return std::numeric_limits<double>::infinity(); // Invalid routing
    }

    double f_mlu = 0.;
    most_congested_arc_id = -1;
    nt::Pair< Arc, double >  most = sr.mostLoadedArc(inst.capacities) ;
    f_mlu = most.second ;
    most_congested_arc_id = inst.network.id(most.first);

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

void Heuristic::computeAllPairsShortestPaths(NetworkPrecompute& precomp) {
    const int N = inst.network.nodeNum();
    precomp.dist_matrix.assign(N, std::vector<double>(N, std::numeric_limits<double>::infinity()));

    // Initialize diagonal to 0
    for (int i = 0; i < N; ++i) {
        precomp.dist_matrix[i][i] = 0.0;
    }

    // Build arc weight map: weight(a) = 1.0 / capacity(a)
    // Prefers high-capacity arcs — routes naturally avoid thin links
    typename Digraph::template ArcMap<double> arc_weight(inst.network);

    for (ArcIt a(inst.network); a != nt::INVALID; ++a) {
        double cap = inst.capacities[a];
        arc_weight[a] = (cap > 0.0) ? (1.0 / cap) : std::numeric_limits<double>::infinity();
    }

    // Dijkstra from every source node
    // Constructor requires both graph AND length_map
    nt::graphs::Dijkstra<Digraph, double> dijkstra(inst.network, arc_weight);

    for (NodeIt src(inst.network); src != nt::INVALID; ++src) {
        int s_id = inst.network.id(src);
        dijkstra.run(src);

        for (NodeIt dst(inst.network); dst != nt::INVALID; ++dst) {
            int d_id = inst.network.id(dst);
            if (dijkstra.reached(dst))
                precomp.dist_matrix[s_id][d_id] = dijkstra.dist(dst);
        }
    }
}


bool Heuristic::RandomHeuristicRun() {
    result_builder.setValid(true);
    int i_total_segments = 0;      // Accumulate total number of segments across all SR paths
    int i_num_srpaths = 0;   // Count only srpaths with more than one segment
    int total_cost = 0 ;
    static std::mt19937 rng(std::random_device{}());
    // ===== LOOP TO ADD WAYPOINTS =====

    for (int t = 0; t < inst.i_num_time_slots; ++t) {
        std::cout << "=== Time Slot " << t << " with " << inst.demand_graph.arcNum() << "demand arcs" << std::endl;
        // Iterate through all demands
             for (int i = 0; i < inst.demand_graph.arcNum(); i++) {
                 // Get the SR path for this time slot and demand

                 DemandArc demand_arc = inst.demand_graph.arcFromId(i);
                 SrPathBit & path = rs.getSrPath(t, demand_arc);
                 int i_num_segments = 0;
                 Node source = inst.demand_graph.source(demand_arc);
                 Node target = inst.demand_graph.target(demand_arc);
                 // Initialize with direct path
                 if (t==0) {
                     path.init(inst.network, 3);  // 10 = estimated capacity for waypoints
                     path.addSegment(source);
                     path.finalize(target);
                 }
                 else {
                     path.copyFrom(rs.getSrPath(t-1, demand_arc));
                 }
                 if ((t==0) || (t>0 && total_cost < scenario.budget[t])) {
                    int worst_arc;
                    double best_mlu = computeMLU(t, rs, worst_arc);
                    SrPathBit best_path;
                    best_path.copyFrom(path);
                    // Try several random waypoints
                    const int num_attempts = 5;  // Adjust based on time constraints
                     int costbestAttempt = 0 ;
                    for (int attempt = 0; attempt < num_attempts; ++attempt) {

                        int wayPointIndex ;
                        do {
                            wayPointIndex = genRandomInt(inst.network.nodeNum());
                        }
                        while(wayPointIndex == inst.network.id(source)|| wayPointIndex == inst.network.id(target)) ;
                        Node p_wp = inst.network.nodeFromId(wayPointIndex);
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
                                    int costPath = dist(rs.getSrPath(t - 1, demand_arc), path);
                                    if ((total_cost-costbestAttempt) + costPath <= scenario.budget[t]) {
                                        total_cost -= costbestAttempt ;
                                        costbestAttempt = dist(rs.getSrPath(t - 1, demand_arc), best_path) ;
                                        best_mlu = current_mlu;
                                        best_path.copyFrom(path);
                                        cout << "It is improved at t " << t << " for demand " <<
                                            inst.demand_graph.id(demand_arc) <<" and for arc " << worst_arc <<
                                                " and MLU is " << best_mlu << endl  ;
                                        total_cost += costPath;
                                        cout << " and total cost for demand : " << inst.demand_graph.id(demand_arc) << " is " << total_cost << endl ;
                                    }
                                }

                        }
                    }
                    // copy the best path into the main solution

                    rs.setSrPaths(t,demand_arc, std::move(best_path));
                    i_num_segments = rs.getSrPath(t,demand_arc).segmentNum() - 1;
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
    return true;
}




bool Heuristic::ArcJumpHeuristicRun() {
    result_builder.setValid(true);
    vector<int> costInterventions(inst.demand_graph.arcNum(),0);
    std::vector<vector<int>> nbSegmentsByDemand(inst.i_num_time_slots, vector<int>(inst.demand_graph.arcNum(), 1));
    int total_cost = 0;
    static std::mt19937 rng(std::random_device{}());

    for (int t = 0; t < inst.i_num_time_slots; ++t) {

        std::cout << "=== Time Slot " << t << std::endl;

        // Initial direct paths
        for (int i = 0; i < inst.demand_graph.arcNum(); i++) {
            DemandArc demand_arc = inst.demand_graph.arcFromId(i);
            SrPathBit & path = rs.getSrPath(t, demand_arc);
            Node source = inst.demand_graph.source(demand_arc);
            Node target = inst.demand_graph.target(demand_arc);
            // Initialize with direct path
            if (t==0) {
                path.init(inst.network, 3);  // 10 = estimated capacity for waypoints
                path.addSegment(source);
                path.finalize(target);
            }
            else {
                path.copyFrom(rs.getSrPath(t-1, demand_arc));
            }
        }

        int max_iterations = 100;

        while (max_iterations--) {

            int worst_arc_id;
            double best_mlu = computeMLU(t, rs, worst_arc_id);

            if (worst_arc_id == -1) break;

            Arc worst_arc = inst.network.arcFromId(worst_arc_id);
            Node u = inst.network.source(worst_arc);
            Node v = inst.network.target(worst_arc);

            // Collect neighbors of u (excluding v)
            std::vector<Node> neighbors;
            for (OutArcIt a(inst.network, u); a != nt::INVALID; ++a) {
                Node neigh = inst.network.target(a);
                if (neigh != v) {
                    neighbors.push_back(neigh);
                }

            }

            if (neighbors.empty()) continue;

            // Pick random neighbor as waypoint
            int RandomWayPoint = genRandomInt(neighbors.size());
            Node wp = neighbors[RandomWayPoint];

            // 👉 Instead of all demands: try a few random ones
            const int trials = 2000;
            int k = 0 ;
            int tmp_arc = worst_arc_id ;
            bool improved = false;
            while (k < trials && improved == false && tmp_arc == worst_arc_id) {

                int i = genRandomInt(inst.demand_graph.arcNum());
                DemandArc demand_arc = inst.demand_graph.arcFromId(i);
                Node s = inst.demand_graph.source(demand_arc);
                Node d = inst.demand_graph.target(demand_arc);

                SrPathBit &path = rs.getSrPath(t, demand_arc);
                SrPathBit backup;
                backup.copyFrom(path);

                // Try rerouting via wp
                path.init(inst.network, 3);
                path.addSegment(s);
                path.addSegment(wp, s);
                path.finalize(d);
                int wArc ;
                double new_mlu = computeMLU(t, rs, wArc);
                int cost = (t == 0) ? 0 :
                dist(rs.getSrPath(t - 1, demand_arc), path);
                if (new_mlu < best_mlu) {
                    if (t == 0 || (  total_cost-costInterventions[i]) + cost <= scenario.budget[t]) {
                        best_mlu = new_mlu;
                        if (t > 0)  {
                            total_cost-=costInterventions[i];
                            total_cost += cost;
                            costInterventions[i] = cost ;
                        }
                        tmp_arc = wArc ;
                        std::cout << "Improved MLU: " << best_mlu << " from demand " << i
                                  << " via arc " << tmp_arc << " total cost is " <<total_cost<<
                                      std::endl;
                        improved = true ;

                    }
                    else {
                        rs.setSrPaths(t,demand_arc, std::move(backup));
                        //path.copyFrom(backup);
                    }

                }
                else {
                    //path.copyFrom(backup);
                    rs.setSrPaths(t,demand_arc, std::move(backup));
                }
                int i_num_segments = rs.getSrPath(t,demand_arc).segmentNum() - 1;
                nbSegmentsByDemand[t][i] = i_num_segments ;
                k++;
            }

        }

        // Budget fallback

        if (t > 0 && total_cost >= scenario.budget[t]) {
            cout << "Budget reached or excessed " << endl ;
            break ;
        }

    }

    // ===== END OF LOOP =====
    if (!simulateSegmentRouting(inst, scenario, rs, result_builder)) {
        result_builder.setValid(false);
        result_builder.display(i_max_decimal_places);
        return false;
    }

    cout << "Results:" << endl;
    result_builder._i_total_segments = 0 ;
    result_builder._i_total_srpaths = 0 ;
    for(int i = 0 ; i < nbSegmentsByDemand.size() ; i++) {
        for (int j = 0 ; j < nbSegmentsByDemand[i].size() ; j++) {
            result_builder._i_total_segments += nbSegmentsByDemand[i][j];
            result_builder._i_total_srpaths += (nbSegmentsByDemand[i][j]>1);
        }

    }
    result_builder._i_total_cost = total_cost;
    std::sort(
    result_builder._sat_values.begin(),
    result_builder._sat_values.end(),
    [](const auto& a, const auto& b) {
        return a.third > b.third; // Sort descending by saturation
    }
    );

    return true;
}

// ===========================
// New targeted heuristic
// ===========================



bool Heuristic::newHeuristicRun() {
    result_builder.setValid(true);
    vector<int> costInterventions(inst.demand_graph.arcNum(),0);
    std::vector<vector<int>> nbSegmentsByDemand(inst.i_num_time_slots, vector<int>(inst.demand_graph.arcNum(), 1));
    int total_cost = 0;
    static std::mt19937 rng(std::random_device{}());

    for (int t = 0; t < inst.i_num_time_slots; ++t) {
        InterventionGuard intervention_guard(inst.network, inst.metrics, scenario.interventions[t]);
        std::cout << "=== Time Slot " << t << std::endl;

        // Initial direct paths
        for (int i = 0; i < inst.demand_graph.arcNum(); i++) {
            DemandArc demand_arc = inst.demand_graph.arcFromId(i);
            SrPathBit & path = rs.getSrPath(t, demand_arc);
            Node source = inst.demand_graph.source(demand_arc);
            Node target = inst.demand_graph.target(demand_arc);
            // Initialize with direct path
            if (t==0) {
                path.init(inst.network, 3);  // 10 = estimated capacity for waypoints
                path.addSegment(source);
                path.finalize(target);
            }
            else {
                path.copyFrom(rs.getSrPath(t-1, demand_arc));
            }
        }

        int max_iterations = 1000;

        while (max_iterations--) {
            // The route  all the sr paths and get the objective
            sr.clear();
            int worst_arc_id;
            Digraph::ArcMap<DemandArray> dpa(inst.network);
            sr.run(inst.demand_graph, inst.dvms[t], rs.getSrPathsAt(t),dpa);
            auto most = sr.mostLoadedArc(inst.capacities);
            double best_mlu = most.second;
            worst_arc_id = (most.first == nt::INVALID) ? -1 : Digraph::id(most.first);
            // end routing

            if (worst_arc_id == -1) break;

            Arc worst_arc = inst.network.arcFromId(worst_arc_id);
            Node u = inst.network.source(worst_arc);
            Node v = inst.network.target(worst_arc);

            // Collect neighbors of u (excluding v)
            std::vector<Node> neighbors;
            for (OutArcIt a(inst.network, u); a != nt::INVALID; ++a) {
                Node neigh = inst.network.target(a);
                if (neigh != v) {
                    neighbors.push_back(neigh);
                    for (OutArcIt b(inst.network, neigh); b != nt::INVALID; ++b) {
                        Node neigh2 = inst.network.target(b);
                        if (neigh2 != neigh && neigh2 != v) {
                            neighbors.push_back(neigh2);
                        }
                    }
                }
            }

            if (neighbors.empty()) continue;

            // Pick random neighbor as waypoint
            int RandomWayPoint = genRandomInt(neighbors.size());
            Node wp = neighbors[RandomWayPoint];

            // 👉 Instead of all demands: try a few random ones

            int k = 0 ;
            int tmp_arc = worst_arc_id ;
            bool improved = false;
            // get the demands that are responsible for the most congested arc
            DemandArray &users = dpa[worst_arc];
            cout << "dpa.size() " << users.size() << " " << "worst_arc_id " << worst_arc_id <<  endl;

            if (users.size() == 0) continue;
            int nb_users = users.size();

            while (k < nb_users  && (improved == false || tmp_arc == worst_arc_id)) {
                DemandArc demand_arc = users[k];
                int i = inst.demand_graph.id(demand_arc);
                Node s = inst.demand_graph.source(demand_arc);
                Node d = inst.demand_graph.target(demand_arc);

                SrPathBit &path = rs.getSrPath(t, demand_arc);
                SrPathBit backup;
                backup.copyFrom(path);

                // Try rerouting via wp
                path.init(inst.network, 3);
                path.addSegment(s);
                path.addSegment(wp, s);
                path.finalize(d);
                int wArc ;
                double new_mlu = computeMLU(sr, t,  tmp_arc, demand_arc,backup, path, dpa, false );
                int cost = (t == 0) ? 0 :
                dist(rs.getSrPath(t - 1, demand_arc), path);
                if (new_mlu < best_mlu) {
                    if (t == 0 || (  total_cost-costInterventions[i]) + cost <= scenario.budget[t]) {
                        best_mlu = computeMLU(sr, t,  tmp_arc, demand_arc,backup, path, dpa, true );
                        if (t > 0)  {
                            total_cost-=costInterventions[i];
                            total_cost += cost;
                            costInterventions[i] = cost ;
                        }
                        //tmp_arc = wArc ;
                        std::cout << "Improved MLU: " << best_mlu << " from demand " << i
                                  << " via arc " << tmp_arc << " total cost is " <<total_cost<<
                                      std::endl;
                        improved = true ;

                    }
                    else {
                        rs.setSrPaths(t,demand_arc, std::move(backup));
                        //path.copyFrom(backup);
                    }

                }
                else {
                    //path.copyFrom(backup);
                    rs.setSrPaths(t,demand_arc, std::move(backup));
                }
                int i_num_segments = rs.getSrPath(t,demand_arc).segmentNum() - 1;
                nbSegmentsByDemand[t][i] = i_num_segments ;
                k++;
            }

        }

        // Budget fallback

        if (t > 0 && total_cost >= scenario.budget[t]) {
            cout << "Budget reached or excessed " << endl ;
            break ;
        }

    }

    // ===== END OF LOOP =====
    if (!simulateSegmentRouting(inst, scenario, rs, result_builder)) {
        result_builder.setValid(false);
        result_builder.display(i_max_decimal_places);
        return false;
    }

    cout << "Results:" << endl;
    result_builder._i_total_segments = 0 ;
    result_builder._i_total_srpaths = 0 ;
    for(int i = 0 ; i < nbSegmentsByDemand.size() ; i++) {
        for (int j = 0 ; j < nbSegmentsByDemand[i].size() ; j++) {
            result_builder._i_total_segments += nbSegmentsByDemand[i][j];
            result_builder._i_total_srpaths += (nbSegmentsByDemand[i][j]>1);
        }

    }
    result_builder._i_total_cost = total_cost;
    std::sort(
    result_builder._sat_values.begin(),
    result_builder._sat_values.end(),
    [](const auto& a, const auto& b) {
        return a.third > b.third; // Sort descending by saturation
    }
    );

    return true;
}
