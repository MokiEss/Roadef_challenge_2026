// main.cpp

#include "Heuristic.h"
#include "readerInstance.h"




int main() {
    string nInstance; cout << "Instance number " ; cin >> nInstance ;

    string net_file = "../challenge-roadef-2026-main/setA/setA-" + nInstance + "-net.json";
    string scenario_file = "../challenge-roadef-2026-main/setA/setA-" + nInstance + "-scenario.json";
    string tm_file = "../challenge-roadef-2026-main/setA/setA-" + nInstance + "-tm.json";

    Instance      inst;
    bool use_ftxui = false ;
    ResultBuilder result_builder(inst, use_ftxui);
    Scenario scenario;
    ReadNetworkInstance( net_file,  scenario_file,   tm_file,  inst,
         use_ftxui,  result_builder, scenario );

    Heuristic hr(inst, use_ftxui, result_builder, scenario );
    hr.RandomHeuristicRun();
    // Validate budget constraints
    if (!checkBudgetConstraint(hr.rs, hr.inst, hr.scenario, hr.result_builder._i_total_cost)) {
        cout << "solution not feasible" << endl ;
        result_builder.setValid(false);
        result_builder.display(12);
        return -1;
    }
    return 0;
}