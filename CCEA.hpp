#ifndef CCEA_HPP
#define CCEA_HPP

#include "NN.hpp"
#include <vector>
#include <map>

class CCEA
{
	private:
		// Number of hidden layer nodes for each NN
		int numHidden;
		// The number of populations in the CCEA
		int numPops;
		// The size of the population after each selection
		int popSize;
		// The standard deviation from mean 0 to change weights
		double stdDev;
		// The percentage of "radiation" to give each NN during mutation
		// At 0%, no weights are changed. At 100%, all weights are modified
		double percRad;
		// The current population of nerual networks
		vector< vector<NN> > populations;
		// The teams formed from the populations
		vector< vector<NN> > teams;

		void RemoveLoserStates(vector< vector<int> > allLosers);
	public:
		// Constructor specifies the number of populations, population size of each pop, 
		// and the standard deviation used for modifying the weights
		CCEA(int numPops, int popSize, double sd, double percRad);

		void Init(const vector<int>, const vector<int>, const vector<int>);
		void Mutate();
		void SelectFittest();
		void CreateTeams();
		void ReformPopulations();

		// THIS IS ALMOST CERTAINLY LOOKED DOWN UPON BY THE GODS OF C++
		// 		Forgive me. I am not worthy.

		// The states of the agents
		// first dimension is the population
		// second dimension is the individual
		// third dimension contain the states for said individual
		vector< vector< vector<double> > > states;
		vector< vector< vector<double> * > > statesPtrs;

		vector< vector<double> > GetOutputsOfTeam(const int t);
		// assign fitness to team members
		void AssignFitness(const int t, const vector<double> fitnesses);
};

#endif
