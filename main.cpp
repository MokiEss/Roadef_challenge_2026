// main.cpp
#include <iostream>
#include "networktools.h"

using Digraph = nt::graphs::SmartDigraph;
using DemandGraph = nt::graphs::DemandGraph<Digraph>;
using DynamicShortestPathRouting = nt::te::DynamicShortestPathRouting<Digraph, double, double>;

int main() {
    // Create a network topology with metrics and capacities
    Digraph network;
    Digraph::ArcMap<double> metrics(network);
    Digraph::ArcMap<double> capacities(network);

    // Add nodes (routers)
    Digraph::Node n1 = network.addNode();
    Digraph::Node n2 = network.addNode();
    Digraph::Node n3 = network.addNode();

    // Add arcs (IP links) with the associated ISIS metrics and capacities
    Digraph::Arc a12 = network.addArc(n1, n2); metrics[a12] = 10; capacities[a12] = 100;
    Digraph::Arc a23 = network.addArc(n2, n3); metrics[a23] = 10; capacities[a23] = 100;
    Digraph::Arc a13 = network.addArc(n1, n3); metrics[a13] = 20; capacities[a13] = 100;

    // Create demand graph
    DemandGraph demands(network);
    DemandGraph::ArcMap<double> volumes(demands);

    // Add a demand from n1 to n3 with volume 50
    DemandGraph::Arc d = demands.addArc(n1, n3); volumes[d] = 50.0;

    // Route the demand in the network according to the shortest path routing protocol with ECMP rule
    DynamicShortestPathRouting dspr(network, metrics);
    dspr.run(demands, volumes);

    // Display the maximum link utilization (MLU)
    // Initial MLU: 0.25 (traffic via n1→n2→n3: 25/100 and n1→n3: 25/100, ECMP applies)
    std::cout << "Initial MLU: " << dspr.maxSaturation(capacities) << std::endl;

    // Save current network state using snapshot
    DynamicShortestPathRouting::Snapshot snapshot(dspr);
    snapshot.save();

    // Simulate link failure by increasing metric to infinity
    dspr.updateArcWeight(a23, DynamicShortestPathRouting::infinity);

    // After failure MLU: 0.5 (traffic rerouted via n1→n3: 50/100, no ECMP)
    std::cout << "After failure MLU: " << dspr.maxSaturation(capacities) << std::endl;

    // Rollback changes using snapshot
    snapshot.restore();
    // After restore MLU: 0.25 (back to original routing)
    std::cout << "After restore MLU: " << dspr.maxSaturation(capacities) << std::endl;
    std::cout << "Im here" ;
    return 0;
}