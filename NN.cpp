#include "NN.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

NN::NN()
{
	NN(0, 0, 0);
}

// Constructor
NN::NN(int inputs, int hidden, int outputs)
{
	// initialize random seed
//	srand(time(NULL));

	// Assign private member variables
	numInputs = inputs;
	numHidden = hidden;
	numOutputs = outputs;
	numInnerWeights = (numInputs + 1)*numHidden;
	numOuterWeights = (numHidden + 1)*numOutputs;

	// Create the weights
	for (int w = 0; w < numInnerWeights + numOuterWeights; w++)
	{
		weights.push_back((double)rand()/(double)RAND_MAX);
	}

	// Create the nodes
	for (int i = 0; i < numInputs; i++)
	{
		inputNodes.push_back(0);
	}

	// One for bias. This one won't change value.
	inputNodes.push_back(1);

	for (int j = 0; j < numHidden; j++)
	{
		hiddenNodes.push_back(0);
	}

	// One for bias. This one won't change value.
	hiddenNodes.push_back(1);

	for (int k = 0; k < numOutputs; k++)
	{
		outputNodes.push_back(0);
	}

//	fitness = 0;
}

// Returns the weight specified by an index
double NN::GetWeight(int w)
{
	return weights[w];
}

// Sets a weight specified by an index to a specific value
void NN::SetWeight(int w, double weight)
{
	weights[w] = weight;
}

int NN::GetNumWeights()
{
	return numInnerWeights + numOuterWeights;
}

double NN::getInnerWeight(int inputNode, int hiddenNode)
{
	// (incl. bias) to the first hidden node are listed first, and so on...
	//return innerWeights[inputNode + hiddenNode*(numHidden + 1)];
	return weights[inputNode + hiddenNode*(numInputs + 1)];
}

void NN::setInnerWeight(int inputNode, int hiddenNode, double newWeight)
{
	// The weight list is arranged such that the weights from all input nodes
	// (incl. bias) to the first hidden node are listed first, and so on...
	weights[inputNode + hiddenNode*(numInputs + 1)] = newWeight;
}

double NN::getOuterWeight(int hiddenNode, int outputNode)
{
	// The weight list is arranged such that the weights from all hidden nodes
	// (incl. bias) to the first output node are listed first, and so on...
	return weights[numInnerWeights + hiddenNode + outputNode*(numHidden + 1)];
}

void NN::setOuterWeight(int hiddenNode, int outputNode, double newWeight)
{
	// The weight list is arranged such that the weights from all hidden nodes
	// (incl. bias) to the first output node are listed first, and so on...
	weights[numInnerWeights + hiddenNode + outputNode*(numHidden + 1)] = newWeight;
}

// Sets the specified weight to a random value between 0 and 1
// (since weights can be any real number, I don't know if this 
// will be a problem later...)
void NN::RandomizeWeight(int weight)
{
	double randWeight = (double)rand()/(double)RAND_MAX;
	weights[weight] = randWeight;
}

// Calculates the sigmoid of a number
double NN::Sigmoid(double x)
{
	return 1.0/(1.0 + exp(-x));
}

double NN::HyperbolicTan(double x)
{
	double z = exp(-2*x);
	return (1 - z)/(1 + z);
}

// Resets all the weights to a random value
void NN::Reset()
{
	for (int w = 0; w < numInnerWeights + numOuterWeights; w++)
	{
		RandomizeWeight(w);
	}
}

// Copy outputs from NN and return
vector<double> NN::GetOutputs()
{
	vector<double> outputs;
	for (int i = 0; i < numOutputs; i++)
		outputs.push_back(outputNodes[i]);
	return outputs;
}

/* Takes a list of inputs and feeds them through the NN
 * INPUTS: 			the vector of inputs
 * RETURNS:			nothing
 * SIDE EFFECTS:	the output of the NN is updated
 */
void NN::FeedForward(vector<double> inputs)
{
	// Load input nodes with input data
	for (int i = 0; i < numInputs; i++)
	{
		// Yeah, for now I'm just gonna think using hyperbolic tangent to scale the inputs
		// is a good idea. We'll see how wrong I am later. :)
		inputNodes[i] = HyperbolicTan(inputs[i]);
	}

	// Iterate through hidden nodes
	for (int j = 0; j < numHidden; j++)
	{
		// Keep track of sum for activation function
		double sum = 0;

		// Multiply input values (incl. bias) with the corresponding weights
		for (int i = 0; i < numInputs + 1; i++)
		{
			// Add to sum
			sum += inputNodes[i]*getInnerWeight(i, j);
		}

		// Update output of jth hidden node
		hiddenNodes[j] = Sigmoid(sum);
	}

	// Iterate through output nodes
	for (int k = 0; k < numOutputs; k++)
	{
		// Keep track of sum for activation function
		double sum = 0;

		// Multiply hidden nodes (incl. bias) with the corresponding weights
		for (int j = 0; j < numHidden + 1; j++)
		{
			// Add to sum
			sum += hiddenNodes[j]*getOuterWeight(j, k);
		}

		// Update output of jth output node
		outputNodes[k] = Sigmoid(sum);
	}
}

// Makes a copy of the current NN and returns it
NN NN::Clone()
{
	NN nn(numInputs, numHidden, numOutputs);
	// Copy weights
	for (int w = 0; w < numInnerWeights + numOuterWeights; w++)
	{
		nn.weights[w] = weights[w];
	}

	return nn;
}

int NN::GetNumInputs()
{
	return numInputs;
}
int NN::GetNumHidden()
{
	return numHidden;
}
int NN::GetNumOutputs()
{
	return numOutputs;
}
