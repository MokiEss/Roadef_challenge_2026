//
// Created by uha on 26/03/2026.
//

#ifndef ROADEF_CHALLENGE_2026_HEURISTIC_H
#define ROADEF_CHALLENGE_2026_HEURISTIC_H
#include "readerInstance.h"

class Heuristic {

    Instance   &   inst;
    bool use_ftxui = false ;
    ResultBuilder & result_builder;
    Scenario & scenario;
    RoutingScheme rs;
    int i_max_decimal_places = 12;
public:

    Heuristic(Instance   &   inst, bool use_ftxui, ResultBuilder & result_builder, Scenario & scenario ):inst(inst),use_ftxui(use_ftxui),
            result_builder(result_builder),scenario(scenario),rs(inst) {} ;
    bool run();
};


#endif //ROADEF_CHALLENGE_2026_HEURISTIC_H