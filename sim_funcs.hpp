#ifndef SIM_FUNCS_HPP
#define SIM_FUNCS_HPP

#define _USE_MATH_DEFINES
#include "sim_structs.hpp"
#include "CCEA.hpp"

sData load_data();

sBattery design_battery(const cv::Mat actions, const cv::Mat batteryData);

sBattery create_battery(sCell cell, double sConfigs, double pConfigs);

sMotor design_motor(const cv::Mat actions, const cv::Mat motorData);

sFoil design_foil(const cv::Mat actions, const cv::Mat foilData);

sProp design_prop(const cv::Mat actions, const cv::Mat foilData);

sProp create_prop(const double diameter, const double angleRoot, const double angleTip, const double chordRoot,
					const double chordTip, const double mass, const double cost);

sRod design_rod(const cv::Mat actions, const cv::Mat matData, const sProp prop);

sRod create_rod(const sMaterial material, const double length, const double diameter, const double thickness);

sSys design_sys(const sBattery battery, const sMotor motor, const sFoil foil, const sProp prop, const sRod rod);

void update_states(std::vector< std::vector<double> * > statesPtrs, const sHover hover, const cv::Mat constraints, const sSys sys);

cv::Mat get_actions(const int t, CCEA* ccea);

void write_propfile(const sProp prop, const sFoil foil);

cv::Mat calc_constraints(const sSys sys, const sHover hover, const int fail);

void counter_calc(const sData data, sCell & avgCell, sMotor & avgMotor, sProp & avgProp, sFoil & avgFoil, sRod & avgRod);

cv::Mat compute_rewards(const int D, const sPenalty penalty, const sSys sys, const sData data, 
					double & G, double & flightTime, cv::Mat & constraints, sHover & hover);

double calc_G(const sPenalty penalty, const sSys sys);

double calc_G(const sPenalty penalty, const sSys sys, double & flightTime, cv::Mat & constraints, sHover & hover);

sPerf call_qprop(std::string velStr, std::string rpmStr, std::string voltStr, std::string dBetaStr, std::string thrustStr, std::string torqueStr, std::string ampsStr, std::string peleStr, std::string mode, int motorNum);

sHover calc_hover(const sSys sys);

void run_experiment(sPenalty penalty, int numGens, int numRuns, int popSize, int D);

#endif
