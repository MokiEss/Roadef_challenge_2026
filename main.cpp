// main.cpp

#include "Heuristic.h"
#include "readerInstance.h"




int main() {
    string net_file = "/home/uha/Desktop/Roadef_challenge_2026/challenge-roadef-2026-main/setA/setA-01-net.json";
    string scenario_file = "/home/uha/Desktop/Roadef_challenge_2026/challenge-roadef-2026-main/setA/setA-01-scenario.json";
    string tm_file = "/home/uha/Desktop/Roadef_challenge_2026/challenge-roadef-2026-main/setA/setA-01-tm.json";

    Instance      inst;
    bool use_ftxui = false ;
    ResultBuilder result_builder(inst, use_ftxui);
    Scenario scenario;
    ReadNetworkInstance( net_file,  scenario_file,   tm_file,  inst,
         use_ftxui,  result_builder, scenario );

    Heuristic hr(inst, use_ftxui, result_builder, scenario );
    hr.RandomHeuristicRun();
    return 0;
}