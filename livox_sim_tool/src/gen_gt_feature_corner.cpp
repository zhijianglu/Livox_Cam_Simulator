//
// Created by lab on 2021/10/21.
//
#include "functions.h"
#include "global.h"
#include "gen_gt_feature_corner.h"


int main(int argc, char *argv[])
{
    readParameters("/home/lab/Calibrate/GazeboSim/livox_sim_tool/src/cfg/config_sim.yaml");
    gen_features();
}
#pragma clang diagnostic pop