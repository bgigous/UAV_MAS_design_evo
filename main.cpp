#include "sim_funcs.hpp"
#include <iostream>
#include "funcs.hpp"

using namespace std;

int main()
{
	tic();

	int numGens = 10;
	int numRuns = 10;
	int popSize = 10;
	
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

	int D = 1;

	run_experiment(penalty, numGens, numRuns, popSize, D);

	double elapsed = toc();
	cout << "Elapsed time: " << elapsed << endl;
}
