#include "sim_funcs.hpp"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include "funcs.hpp"

using namespace std;

int main(int argc, char* argv[])
{
	tic();
	srand(time(NULL));

	if (argc != 6)
	{
		cout << "Wrong number of inputs. Usage:" << endl;
		cout << argv[0] << " ";
		cout << "GENERATIONS RUNS POPSIZE USE_D STATEMODE" << endl;
		cout << "STATEMODE: 0 - random, 1 - simple constraint, 2 - experimental, 3 - 2, but with ""upate""" << endl;
	}

	int numGens = 	atoi(argv[1]);
	int numRuns = 	atoi(argv[2]);
	int popSize = 	atoi(argv[3]);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	int D = 		atoi(argv[4]);
	int stateMode = atoi(argv[5]);
	
	if (stateMode > 3)
		cout << "StateMode " << stateMode << " probably doesn't exist yet... BEWARE!" << endl;
	
	// const quad death deathplus lin none
	sPenalty penalty;
	penalty.Mode = "quad";
	penalty.quadMin = 100;
	penalty.quadMax = 100;
	penalty.quadtrunc = -100;
	penalty.Const = 100;
	penalty.death = -100;
	penalty.lin = -1000;
	penalty.failure = -100;

	run_experiment(penalty, numGens, numRuns, popSize, D, stateMode);

	/*
	sData data = load_data();
	cv::Mat actions = (cv::Mat_<double>(13, 1) << 0, 0, 0, 0, 1, 0.2, 0.3, 0.23, 0.52, 0.01, 2, 0.89, 0.91);
	sMaterial mat = {
				.Type = 2,
				.Ymod = 100,
				.Sut = 620,
				.Sy = 500,
				.Dens = 4507,
				.Cost = 8.848*100000
			};
	double Length = 0.3217, Dia = 0.00635, Thick = 0.001651;
	sBattery battery = {	
			.Cell = {
				.Cost = 7.97,
				.Cap = 5,
				.C = 20,
				.Mass = 0.114
			},
			.sConfigs = 3,
			.pConfigs = 1,
			.Cost = 23.91,
			.Mass = 0.342,
			.Volt = 11.1,
			.Cap = 5,
			.C = 20,
			.Imax = 100,
			.Energy = 1.998*100000
		};
	sMotor motor = design_motor(actions, data.motorData);
	sProp prop = {
			.diameter = 0.4064,
			.angleRoot = 20,
			.angleTip = 30,
			.chordRoot = 0.00254,
			.chordTip = 0.00508,
			.mass = 2.808*1/10000.0,
			.cost = 0.00417
		};
	sFoil foil = design_foil(actions, data.foilData);
	sRod rod = create_rod(mat, Length, Dia, Thick);
	sSys sys = design_sys(battery, motor, foil, prop, rod);

	write_propfile(prop, foil);
	
	sHover hover = calc_hover(sys);

	cout << calc_constraints(sys, hover, hover.failure) << endl;
	sPerf hover = {
		(cv::Mat_<double>(1, 1) << 0),
		(cv::Mat_<double>(1, 1) << 5095),
		(cv::Mat_<double>(1, 1) << 0),
		(cv::Mat_<double>(1, 1) << 7.148),
		(cv::Mat_<double>(1, 1) << 0.8145),
		(cv::Mat_<double>(1, 1) << 434.6),
		(cv::Mat_<double>(1, 1) << 13.867),
		(cv::Mat_<double>(1, 1) << 35.3175),
		(cv::Mat_<double>(1, 1) << 0.8873),
		(cv::Mat_<double>(1, 1) << 0),
		(cv::Mat_<double>(1, 1) << 0),
		(cv::Mat_<double>(1, 1) << 0.01306),
		(cv::Mat_<double>(1, 1) << 0.008369),
		(cv::Mat_<double>(1, 1) << 10.84),
		(cv::Mat_<double>(1, 1) << 0),
		(cv::Mat_<double>(1, 1) << 489.7),
		(cv::Mat_<double>(1, 1) << 0),
		(cv::Mat_<double>(1, 1) << 1.1685),
		(cv::Mat_<double>(1, 1) << 0.765),
		0
	};
	
	cv::Mat constraints = calc_constraints(sys, hover, 0);
	cout << constraints << endl;
	*/

	/*
	sData data = load_data();
	cv::Mat actions = (cv::Mat_<double>(13, 1) << 0, 0, 0, 4, 3, 0.2, 0.3, 0.23, 0.52, 0.01, 2, 0.89, 0.91);
	sProp prop = design_prop(actions, data.foilData);
	sRod component = design_rod(actions, data.matData, prop);
	*/

	double elapsed = toc();
	cout << "Elapsed time: " << elapsed << endl;
}
