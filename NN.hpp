#ifndef NN_HPP
#define NN_HPP

#include <vector>

using namespace std;

class NN
{
	protected:
		// ID of *this* neural network
		long double ID;
		// The number of input nodes for the neural network
		int numInputs;
		// The number of hidden layer nodes for the neural network
		int numHidden;
		// The number of output nodes for the neural network
		int numOutputs;

		int numInnerWeights;
		int numOuterWeights;

		// The values output from the nodes
		vector<double> inputNodes;
		vector<double> hiddenNodes;
		vector<double> outputNodes;
		// The weights of the synapses from the input layer to the hidden layer
		vector<double> weights;

		double getInnerWeight(int inputNode, int hiddenNode);
		void setInnerWeight(int inputNode, int hiddenNode, double newWeight);
		double getOuterWeight(int hiddenNode, int outputNode);
		void setOuterWeight(int hiddenNode, int outputNode, double newWeight);

		// Sets the specified weight to a random value between 0 and 1
		void RandomizeWeight(int weight);

	private:
		// Calculates the sigmoid of a number
		double Sigmoid(double x);
		double HyperbolicTan(double x);

	public:
		// The NN's fitness (Objective function not included. Adult supervision required)
		double fitness;

		// ID of next neural network
		static long double nextID;

		NN();
		NN(int inputs, int hidden, int outputs);
		// Resets all the weights to a random value
		void Reset();
		// Copy outputs from NN and return
		vector<double> GetOutputs();
		// Takes a list of inputs and feeds them through the NN
		void FeedForward(vector<double> inputs);
		// Makes a copy of the current NN and returns it
		NN Clone();
		// Returns the weight specified by an index
		double GetWeight(int w);
		// Sets a weight specified by an index to a specific value
		void SetWeight(int w, double weight);
		// Get the total number of weights in the NN
		int GetNumWeights();

		// TEMP?
		int GetNumInputs();
		int GetNumHidden();
		int GetNumOutputs();
};

#endif
