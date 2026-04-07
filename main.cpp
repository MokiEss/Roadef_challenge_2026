// main.cpp

#include "Heuristic.h"


#include "readerInstance.h"




int main() {
    string nInstance; cout << "Instance number " ; cin >> nInstance ;
    Instance      inst;
    bool use_ftxui = false ;
    ResultBuilder result_builder(inst, use_ftxui);
    Scenario scenario;
    SegmentRouting sr(inst.network, inst.metrics);
    PreprocessingEngine engine = readAndPreprocess( nInstance, inst, use_ftxui, result_builder, scenario);

    Heuristic hr(inst, use_ftxui, result_builder, scenario, sr );
    //hr.RandomHeuristicRun();
    hr.ArcJumpHeuristicRun();
    //hr.newHeuristicRun();
    // Validate budget constraints

    if (!checkBudgetConstraint(hr.rs, hr.inst, hr.scenario, hr.result_builder._i_total_cost)) {
        cout << "solution not feasible" << endl ;
        hr.result_builder.setValid(false);
        hr.result_builder.display(12);
    }
    else {
        cout << "solution  feasible" << endl ;
        hr.result_builder.setValid(true);
        hr.result_builder.display(12);
    }
    return 0;
}