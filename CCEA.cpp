#include "CCEA.hpp"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <iostream>

#define RAND_DOUBLE ((double)rand()/(double)RAND_MAX)

using namespace std;

vector<int> Range(unsigned int start, unsigned int stop)
{
	vector<int> list;
	for (unsigned int i = start; i < stop; i++)
	{
		list.push_back((int)i);
	}
	return list;
}

vector<int> Shuffle(vector<int> list)
{
	random_shuffle(list.begin(), list.end());
	return list;
}

// Generates a list of unique numbers between 
vector<int> RandomList(unsigned int max, unsigned int length, unsigned int min = 0)
{
	if ((max - min) < length)
	{
		// return empty vector
		vector<int> list;
		return list;
	}

	vector<int> range = Range(min, max);
	vector<int> list;

	for (size_t i = 0; i < length; i++)
	{
		int idx = rand() % range.size();
		list.push_back(range[idx]);
		range.erase(range.begin() + idx);
	}

	return list;
}

double GenerateRandFloat(double mean, double sd)
{
	// Uses Box-Muller transform
	return mean + sd * (sqrt(-2.0 * log(RAND_DOUBLE)) * sin(2.0 * M_PI * RAND_DOUBLE));
}

/* Constructor for the cooperative coevolutionary algorithm
	INPUTS: numHidden - the number of hidden layer nodes for all NNs
			numPops - The number of populations. Corresponds to the number of sector
				agents
			popSize - Number of individual NNs per population
			sd - Standard deviation used when mutating NNs
			percRad - the percentage of weights to alter
*/
CCEA::CCEA(int numPops, int popSize, double sd, double percRad) :
	numPops(numPops), popSize(popSize), stdDev(sd), percRad(percRad)
{
	// Initialize member variables (above)
	// We'll create the populations in the Init() function
}

// TODO implement where all networks for all pops have same params
//void CCEA::Init(const int numInputs, const int numHidden, const int numOutputs)

/*
 * INPUTS: vecNumInputs - a vector of size numPops that gives num inputs for diff pops n stuff
 *  		rewst of input s are sim
 */
void CCEA::Init(const vector<int> vecNumInputs, const vector<int> vecNumHidden, const vector<int> vecNumOutputs)
{
	if ( (int)vecNumInputs.size() != numPops )
	{
		cout << "ERROR CCEA Init(): Vectors aren't the same size!" << endl;
		return;
	}
	else if ( vecNumInputs.size() != vecNumHidden.size() || vecNumInputs.size() != vecNumOutputs.size() )
	{
		cout << "ERROR CCEA Init(): Vectors must be of size equal to the number of populations!" << endl;
		return;
	}

	// Check for positive integers only?

	// Iterate through populations
	for (int n = 0; n < numPops; n++)
	{
		// Create a population
		vector<NN> pop;
		for (int i = 0; i < popSize; i++)
		{
			// Create an individual, with random weights
			NN nn(vecNumInputs[n], vecNumHidden[n], vecNumOutputs[n]);
			// Then add it to the population
			pop.push_back(nn);
		}
		// Add population to list of populations
		populations.push_back(pop);
	}

	// Iterate through populations again to initialize the state things
	// REUSING THIS CODE AND DON'T NEED THIS STATE SHIT?
	// Then DELETE ME. I dare you.
	// But seriously you can get rid of this without any serious repercussions.
	// Concussions are still somewhat likely.
	for (int n = 0; n < numPops; n++)
	{
		vector< vector<double> > indivStates;
		for (int i = 0; i < popSize; i++)
		{
			vector<double> stateVector;
			for (int s = 0; s < vecNumInputs[n]; s++)
				stateVector.push_back((double)rand()/(double)RAND_MAX * 2 - 1);
			indivStates.push_back(stateVector);
		}
		states.push_back(indivStates);
	}

	for (int i = 0; i < popSize*2; i++)
	{
		vector< vector<double> * > teamStatesPtrs;
		for (int n = 0; n < numPops; n++)
		{
			teamStatesPtrs.push_back(NULL);
		}
		statesPtrs.push_back(teamStatesPtrs);
	}
	// DELETE NO FURTHER (or maybe you wanna)	

	// This should go here, right?
	//Mutate();
}

// Mutate all populations
void CCEA::Mutate()
{
	// Iterate through populations (LATER?: use the : notation thing)
	for (int p = 0; p < numPops; p++)
	{
		// Iterate through all individuals of the population
		for (int n = 0; n < popSize*2; n++)
		{
			// Make a copy of the NN to mutate
			NN nnMut = populations[p][n].Clone();
			int numWeights = nnMut.GetNumWeights();

			// Create a list of random (unique) numbers, ranging from 0 to the highest
			// possible weight index.
			// The size of this list depends on how much radiation was specified
			vector<int> weightsToMutate = 
				RandomList(numWeights, (int)(numWeights*percRad));

			// Iterate through this list...
			for (vector<int>::iterator wtIt = weightsToMutate.begin();
				wtIt != weightsToMutate.end();
				wtIt++)
			{
				// And perturb the corresponding weights
				// The higher the standard deviation specified, the more severe the changes
				nnMut.SetWeight(*wtIt, nnMut.GetWeight(*wtIt) + GenerateRandFloat(0, stdDev));
			}

			// Add this mutated copy to the population
			populations[p].push_back(nnMut);
		}
	}

	// DELETE FOR YOUR PLEASURE (rem I put states here against the wishes of the C++ gods)
	// Iterate through populations
	for (int p = 0; p < numPops; p++)
	{
		// Iterate through all individuals of the population
		for (int n = 0; n < popSize; n++)
		{	
			vector<double> myStatesForNow;
			for (size_t s = 0; s < states[p][n].size(); s++)
			{
				// Only want to go through first half! Gonna copy dem states
				// We'll use the same states for the child as the parent
				myStatesForNow.push_back(states[p][n][s]);
			}
			// push new "individual states" thing to the populations of asd states
			states[p].push_back(myStatesForNow);
		}
	}
}

// Using binary tournament, eliminate teams of NNs that don't "match up"
void CCEA::SelectFittest()
{
	vector< vector<int> > allLosers;

	// Iterate through all populations so we can remove entire teams of NNs
	for (vector<vector<NN> >::iterator popIt = populations.begin();
		popIt != populations.end();
		popIt++)
	{
		if (!popIt->empty())
		{
			// Create a list of all the indices of the individuals in the populations
			// and randomly arrange them.
			vector<int> combatants = Shuffle(Range(0, (unsigned int)popSize*2));

			// List of "loser" teams
			vector<int> losers;

			/* I know this looks intimidating, but it's not that bad.
			 * 
			 * n is the number of NNs "removed" from each population thus far
			 * i is the index of the first "combatant" in the binary tournament
			 * j is the index of the second "combatant"
			 */
			for (int n = 0, i = 0, j = 1; n < popSize; n++, i += 2, j += 2)
			{
				// First combatant and second combatant are selected to compete
				int a = combatants[i];
				int b = combatants[j];
				NN nnA = popIt->at(a);
				NN nnB = popIt->at(b);

				// Compare fitnesses and eliminate weaker of the two.
				if 		(nnA.fitness > nnB.fitness) losers.push_back(b);
				else if (nnA.fitness < nnB.fitness) losers.push_back(a);
				else
				{
					// Or if they are equally fit, eliminate one randomly
					if (RAND_DOUBLE >= 0.5)	losers.push_back(b);
					else					losers.push_back(a);
				}
			}

			// Sort the list of losers so they're easier to remove
			sort(losers.begin(), losers.end());

			// Iterate through loser indices (in reverse)
			for (vector<int>::reverse_iterator loserIt = losers.rbegin();
				loserIt != losers.rend();
				loserIt++)
			{
				// Loser elements start with lowest number (now that it's sorted)
				// So we remove losers starting from end
				popIt->erase(popIt->begin() + *loserIt);
			}
			allLosers.push_back(losers);
		}
	}

	//DELETE ME IF you want to LiVE
	// Er, if you don't need it, get rid of and stuff
	RemoveLoserStates(allLosers);
}

void CCEA::RemoveLoserStates(vector< vector<int> > allLosers)
{
	for (int n = 0; n < numPops; n++)
	{
		// Iterate through loser indices (in reverse)
		for (int i = 0; i < popSize; i++)
		{
			// Loser elements start with lowest number (now that it's sorted)
			// So we remove losers starting from end
			states[n].erase(states[n].begin() + allLosers[n][i]);
		}
	}
}

void CCEA::CreateTeams()
{
	teams.clear();
	vector< vector<int> > teamMembers;
	for (int n = 0; n < numPops; n++)
	{
		teamMembers.push_back(RandomList(popSize*2, popSize*2));
	}
	for (int i = 0; i < popSize*2; i++)
	{
		vector<NN> team;
		for (int n = 0; n < numPops; n++)
		{
			team.push_back(populations[n][teamMembers[n][i]]);
		}
		teams.push_back(team);
	}

	for (int i = 0; i < popSize*2; i++)
	{
		for (int n = 0; n < numPops; n++)
		{
			statesPtrs[i][n] = &states[n][teamMembers[n][i]];
		}
	}
}

void CCEA::ReformPopulations()
{
	populations.clear();
	for (int n = 0; n < numPops; n++)
	{
		vector<NN> pop;
		for (int t = 0; t < popSize*2; t++)
		{
			pop.push_back(teams[t][n]);
		}
		populations.push_back(pop);
	}
}

vector< vector<double> > CCEA::GetOutputsOfTeam(const int t)
{
	vector< vector<double> > outputs;
	for (int n = 0; n < numPops; n++)
	{
		teams[t][n].FeedForward(*statesPtrs[t][n]);
		outputs.push_back(teams[t][n].GetOutputs());
	}
	return outputs;
}

void CCEA::AssignFitness(const int t, const vector<double> fitnesses)
{
	if ((int)fitnesses.size() != numPops)
	{
		cout << "ERROR CCEA AssignFitness: not enough fitnesses" << endl;
	}
	
	for (int n = 0; n < numPops; n++)
	{
		teams[t][n].fitness = fitnesses[n];
	}
}
