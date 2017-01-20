#!/usr/bin/env bash

numGens=150
numRuns=50
popSize=5
D=0

for stateMode in 0 1 2 3
do
	./uav_mas $numGens $numRuns $popSize $D $stateMode
	if [[ $? != 0 ]]; then
		break
	fi
done
