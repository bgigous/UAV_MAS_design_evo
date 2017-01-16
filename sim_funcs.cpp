#define NUMAGENTS 13

#include "sim_structs.hpp"
#include "sim_funcs.hpp"
#include "funcs.hpp"
#include "CCEA.hpp"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <algorithm>

sData load_data()
{
	sData data;
	data.batteryData = read_mat_from_file("batterytable.csv");
	data.motorData = read_mat_from_file("motortable.csv");
	data.foilData = read_mat_from_file("airfoiltable.csv");
	data.matData = read_mat_from_file("materialtable.csv");
	return data;
}

sBattery design_battery(const cv::Mat actions, const cv::Mat batteryData)
{
	cv::Mat temp;
	int row = (int)actions.at<double>(0, 0);
	assign_mat(temp, batteryData, -1, -1, -1, -1, row, row, -1, -1);
	sCell cell = {
		temp.at<double>(0, 0),
		temp.at<double>(0, 1)/1000.0,
		temp.at<double>(0, 2),
		temp.at<double>(0, 3)/1000.0
	};

	return create_battery(cell, (int)actions.at<double>(1, 0), (int)actions.at<double>(2, 0));
}

sBattery create_battery(sCell cell, double sConfigs, double pConfigs)
{
	int numCells = sConfigs * pConfigs;
	double Cost = cell.Cost * numCells;
	double Mass = cell.Mass * numCells;
	double Volt = 3.7 * sConfigs;
	double Cap = cell.Cap * pConfigs;
	double C = cell.C;
	double Imax = C * Cap;
	double Energy = Volt * Cap;
	
	sBattery battery = { cell, sConfigs, pConfigs, Cost, Mass, Volt, Cap, C, Imax, Energy };
	return battery;
}

sMotor design_motor(const cv::Mat actions, const cv::Mat motorData)
{
	cv::Mat temp;
	int row = (int)actions.at<double>(3, 0);
	assign_mat(temp, motorData, -1, -1, -1, -1, row, row, -1, -1);
	double kv = temp.at<double>(0, 0);
	double R0 = temp.at<double>(0, 1)/1000.0;
	double I0 = temp.at<double>(0, 2);
	double Imax = temp.at<double>(0, 3);
	double Pmax = temp.at<double>(0, 4);
	double Mass = temp.at<double>(0, 5)/1000.0;
	double Cost = temp.at<double>(0, 6);
	double Diam = temp.at<double>(0, 7)/1000.0;
	double planArea = (M_PI/4) * pow(Diam, 2);
	int Num = row;
	
	sMotor motor = { kv, R0, I0, Imax, Pmax, Mass, Cost, Diam, planArea };
	return motor;
}

sFoil design_foil(const cv::Mat actions, const cv::Mat foilData)
{
	cv::Mat temp;
	int row = (int)actions.at<double>(4, 0);
	assign_mat(temp, foilData, -1, -1, -1, -1, row, row, -1, -1);
	double Cl0 = temp.at<double>(0, 0);
	double Cla = temp.at<double>(0, 1)*360/(2.0*M_PI);
	double Clmin = temp.at<double>(0, 2);
	double Clmax = temp.at<double>(0, 3);
	double Cd0 = temp.at<double>(0, 4);
	double Cd2 = temp.at<double>(0, 5)*360/(2.0*M_PI);
	double Clcd0 = temp.at<double>(0, 6);
	double Reref = temp.at<double>(0, 7);
	double Reexp = temp.at<double>(0, 8);
	int Num = (int)temp.at<double>(0, 9);
	sFoil foil = { Cl0, Cla, Clmin, Clmax, Cd0, Cd2, Clcd0, Reref, Reexp, Num };
	return foil;
}

sProp design_prop(const cv::Mat actions, const cv::Mat foilData)
{
	double diameter = (actions.at<double>(5, 0)*22 + 2)*0.0254;
	double angleRoot = actions.at<double>(6, 0)*45.0;
	double angleTip = actions.at<double>(7, 0)*45.0;
	double chordRoot = (actions.at<double>(8, 0)*1.5 + 0.1)*0.0254;
	double chordTip = (actions.at<double>(9, 0)*1.5 + 0.1)*0.0254;

	double chordAvg = (chordRoot + chordTip)/2.0;
	sFoil foil = design_foil(actions, foilData);

	std::string foilnum = "NACA00" + to_str(foil.Num);
	double avgThickness = 0.01*foil.Num*chordAvg;
	double xsArea = 0.5*avgThickness*chordAvg;
	double vol = xsArea*diameter;
	double mass = vol*1190.0; // assuming polycarb
	double costdens = pow(0.29*(100/2.54), 3); // assuming polycarb
	double cost = costdens*vol;

	return create_prop(diameter, angleRoot, angleTip, chordRoot, chordTip, mass, cost);
}

sProp create_prop(const double diameter, const double angleRoot, const double angleTip, const double chordRoot,
					const double chordTip, const double mass, const double cost)
{
	sProp prop = { diameter, angleRoot, angleTip, chordRoot, chordTip, mass, cost };
	return prop;
}

sRod design_rod(const cv::Mat actions, const cv::Mat matData, const sProp prop)
{
	cv::Mat temp;
	int row = (int)actions.at<double>(10, 0);
	assign_mat(temp, matData, -1, -1, -1, -1, row, row, -1, -1);
	int Type = row;
	double Ymod = temp.at<double>(0, 0);
	double Sut = temp.at<double>(0, 1);
	double Sy = temp.at<double>(0, 2);
	double Dens = temp.at<double>(0, 3);
	double Cost = temp.at<double>(0, 4)*pow(100.0/2.54, 3);
	
	sMaterial mat = { Type, Ymod, Sut, Sy, Dens, Cost };
	
	double sepDist = 0.25*prop.diameter + prop.diameter;
	double motorDist = sepDist/sqrt(2);
	double framewidth = 0.075;
	double minRodLength = motorDist-framewidth/2.0;

	double length = minRodLength;
	double diameter = (actions.at<double>(11, 0)*1.5 + 0.25)*2.54/100.0;
	double thickness = (actions.at<double>(12, 0)*0.25 + 0.035)*2.54/100.0;
	
	return create_rod(mat, length, diameter, thickness);
}

sRod create_rod(const sMaterial material, const double length, const double diameter, const double thickness)
{
	sMaterial mat = material;
	double Length = length;
	double Dia = diameter;
	double Thick = thickness;
	double Area = 0.5*M_PI*(pow(Dia, 2) - pow(Dia - Thick, 2));
	double Amoment = M_PI*(pow(Dia, 4) - pow(Dia - Thick, 4))/64.0;
	double Stiffness = pow(pow(Length, 3)/(3*Amoment*pow(10, 9)*mat.Ymod), -1);
	double Vol = Length*Area;
	double Mass = Vol*mat.Dens;
	double Cost = mat.Cost*Vol;
	double planArea = Length*Dia;
	sRod rod = { mat, Length, Dia, Thick, Area, Amoment, Stiffness, Vol, Mass, Cost, planArea };
}

sSys design_sys(const sBattery battery, const sMotor motor, const sFoil foil, const sProp prop, const sRod rod)
{
	// Move this for speed? Probably doesn't even matter. Move this for organization? Might matter.
	double resMass = 0.3;
	double resFramewidth = 0.075;
	double resPlanArea = pow(resFramewidth, 2);
	double resCost = 50;
	double resPower = 5;
	sRes res = { resMass, resFramewidth, resPlanArea, resCost, resPower };

	double mass = 4.0*motor.Mass + battery.Mass + 4.0*rod.Mass + 4.0*prop.mass + res.mass;
	double planArea = 4.0*rod.planArea + 4.0*motor.planArea + res.planArea;
	double natFreq = sqrt(rod.Stiffness/(0.5*rod.Mass + motor.Mass + prop.mass))/(2*M_PI);
	double cost = 4.0*rod.Cost + 4.0*motor.Cost + battery.Cost + 4.0*prop.cost + res.cost;
	double power = res.power;

	sSys sys = { battery, motor, foil, prop, rod, res, mass, planArea, natFreq, cost, power };
	return sys;
}

void update_states(std::vector< std::vector<double> * > statesPtrs, const sPerf hover, const cv::Mat constraints, const sSys sys)
{
	// Well, if we can prevent updating states for ALL networks, maybe the code would be a bit faster...
	// should do that sometime
	for (int ag = 0; ag < NUMAGENTS; ag++)
	{
		switch (ag)
		{
			case 0:
			case 1:
			case 2:
				statesPtrs[ag]->at(0) = std::max(constraints.ATD(1, 0), 0.0);
				statesPtrs[ag]->at(1) = std::max(constraints.ATD(2, 0), 0.0);
				statesPtrs[ag]->at(2) = std::max(constraints.ATD(3, 0), 0.0);
			case 3:
				statesPtrs[ag]->at(0) = std::max(constraints.ATD(1, 0), 0.0);
				statesPtrs[ag]->at(1) = std::max(constraints.ATD(4, 0), 0.0);
				statesPtrs[ag]->at(2) = std::max(constraints.ATD(5, 0), 0.0);
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
				statesPtrs[ag]->at(0) = std::max(constraints.ATD(1, 0), 0.0);
				statesPtrs[ag]->at(1) = std::max(constraints.ATD(4, 0), 0.0);
				statesPtrs[ag]->at(2) = std::max(constraints.ATD(7, 0), 0.0);
			case 10:
			case 11:
			case 12:
				statesPtrs[ag]->at(0) = std::max(constraints.ATD(1, 0), 0.0);
				statesPtrs[ag]->at(1) = std::max(constraints.ATD(6, 0), 0.0);
				statesPtrs[ag]->at(2) = std::max(constraints.ATD(7, 0), 0.0);
		}
	}
}

cv::Mat get_actions(const int t, CCEA* ccea)
{
	std::vector< std::vector<double> > outputs = ccea->GetOutputsOfTeam(t);

	cv::Mat actions = cv::Mat::zeros(NUMAGENTS, 1, CV_64F);
	for (int ag = 0; ag < NUMAGENTS; ag++)
	{
		switch (ag)
		{
			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
			case 10:
				int actionTemp;// action is number of output which holds max value
				max_double(vector_to_Mat_double(outputs[ag]), actionTemp); 
				actions.ATD(ag, 0) = (double)actionTemp;
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
			case 11:
			case 12:
				actions.ATD(ag, 0) = outputs[ag][0]; // single output for these agents
		}
	}
	return actions;
}

void write_propfile(const sProp prop, const sFoil foil)
{
	int sects = 20;
	double radius = prop.diameter/2.0;
	double root = 0.02;
	cv::Mat radiusvect = linspace(root, radius, sects);
	cv::Mat anglevect = prop.angleRoot + radiusvect*(prop.angleTip - prop.angleRoot)/radius;
	cv::Mat chordvect = prop.angleRoot + radiusvect*(prop.angleTip - prop.angleRoot)/radius;

	ofstream propfile("propfile");
	propfile.setf(ios::fixed, ios::floatfield);
	propfile.precision(6);
	if (propfile.is_open())
	{
		propfile << "Be sure to drink your Ovaltine!\n\n";
		propfile << "2\n\n";
		propfile << foil.Cl0 << " " << foil.Cla << "\n";
		propfile << foil.Clmin << " " << foil.Clmax << "\n\n";
		propfile << foil.Cd0 << " " << foil.Cd2 << " " << foil.Clcd0 << "\n";
		propfile << foil.Reref << " " << foil.Reexp << "\n\n";
		propfile << "1 1 1\n";
		propfile << "0 0 0\n";

		for (int i =0; i < radiusvect.total(); i++)
		{
			propfile << radiusvect.ATD(i, 0) << " " << chordvect.ATD(i, 0) << " " << anglevect.ATD(i, 0) << "\n";
		}

		propfile.close();
	}
	else std::cout << "Well shit, can't open the propfile" << std::endl;
}

cv::Mat calc_constraints(const sSys sys, const sPerf hover, const int fail)
{
	cv::Mat constraints = cv::Mat::zeros(8, 1, CV_64F);
	if (fail)
	{
		constraints.ATD(0, 0) = 10*fail;
	}
	else
	{
		// hover should have only one element for each thing below
		if (hover.volts.total() != 1) std::cout << "HONEY SCONES" << std::endl;
		double thrustReq = sys.mass*9.81/4.0;
		constraints.ATD(1, 0) = 10*(1-hover.thrust.ATD(0,0)/thrustReq); // factor of 10 arbitrary
		constraints.ATD(2, 0) = (4*hover.amps.ATD(0,0))/sys.battery.Imax - 1.0; // perf is from EACH motor
		constraints.ATD(3, 0) = hover.volts.ATD(0,0)/sys.battery.Volt - 1.0;
		constraints.ATD(4, 0) = hover.amps.ATD(0,0)/sys.motor.Imax - 1.0;
		constraints.ATD(5, 0) = hover.pelec.ATD(0,0)/sys.motor.Pmax - 1.0;
		double forcedFreq = hover.rpm.ATD(0,0)/60.0;
		double minnatFreq = 2*forcedFreq;
		constraints.ATD(6, 0) = 1.0 - sys.natFreq/minnatFreq;
		double maxDefl = 0.01*sys.rod.Length;
		double defl = hover.thrust.ATD(0,0)/sys.rod.Stiffness;
		constraints.ATD(7, 0) = defl/maxDefl - 1.0;
	}
	return constraints;
}

void counter_calc(const sData data, sCell & avgCell, sMotor & avgMotor, sProp & avgProp, sFoil & avgFoil, sRod & avgRod)
{
	avgCell.Cost = 6.786666666666666666666666;
	avgCell.Cap = 3.11666666666666666666666;
	avgCell.C = 26.66666666666666666666;
	avgCell.Mass = 0.07665;
	// avgCell.Length = 111.0;
	// avgCell.Width = 38.666666666666666666;
	// avgCell.Height = 8.0;

	avgMotor.kv = 814.58333333333333333;
	avgMotor.R0 = 0.1182916666666666666666;
	avgMotor.I0 = 0.575;
	avgMotor.Imax = 25.14833333333333333;
	avgMotor.Pmax = 461.66666666666666;
	avgMotor.Mass = 0.11225833333333;
	avgMotor.Cost = 68.275;
	avgMotor.Diam = 0.03348958333333333333;
	avgMotor.planArea = 8.80865031625737/10000.0;
	avgMotor.Num = -1;

	avgProp.diameter = 0.5616;
	avgProp.angleRoot = 15.0;
	avgProp.angleTip = 15.0;
	avgProp.chordRoot = 0.0432;
	avgProp.chordTip = 0.0432;
	
	avgFoil.Cl0 = 0;
	avgFoil.Cla = 6.230916022047702;
	avgFoil.Clmin = -1.2875;
	avgFoil.Clmax = 1.2875;
	avgFoil.Cd0 = 0.00675;
	avgFoil.Cd2 = 0.00636699348391;
	avgFoil.Clcd0 = 0;
	avgFoil.Reref = 1000000;
	avgFoil.Reexp = -0.5;
	avgFoil.Num = 14;

	sMaterial avgMat;
	avgMat.Ymod = 43.225;
	avgMat.Sut = 265.2;
	avgMat.Sy = 226.7;
	avgMat.Dens = 2389.75;
	avgMat.Cost = 254316.4535147968;

	avgRod.mat = avgMat;
	avgRod.Length = 0.5334;
	avgRod.Dia = 0.015279;
	avgRod.Thick = 0.00142636875;
}

cv::Mat compute_rewards(const int D, const sPenalty penalty, const sSys sys, const sData data, 
					double & G, double & flightTime, cv::Mat & constraints, sPerf & hover)
{
	cv::Mat rewards;
	G = calc_G(penalty, sys, flightTime, constraints, hover);

	if (!D)
	{
		assign_mat(rewards, (cv::Mat)(cv::Mat::ones(NUMAGENTS, 1, CV_64F) * G));
	}
	else
	{
		assign_mat(rewards, cv::Mat::zeros(NUMAGENTS, 1, CV_64F));

		sCell avgCell;
		sBattery counterBattery;
		sMotor avgMotor;
		sProp avgProp, counterProp;
		sFoil avgFoil;
		sRod avgRod, counterRod;
		counter_calc(data, avgCell, avgMotor, avgProp, avgFoil, avgRod);

		sSys counterSys = sys;
		for (int ag = 0; ag < NUMAGENTS; ag++)
		{
			switch (ag)
			{
				case 0:
					counterBattery = create_battery(avgCell, sys.battery.sConfigs, sys.battery.pConfigs);
					break;
				case 1:
					counterBattery = create_battery(sys.battery.Cell, 2, sys.battery.pConfigs);
					break;
				case 2:
					counterBattery = create_battery(sys.battery.Cell, sys.battery.sConfigs, 1);
					break;
				case 5:
					counterProp = create_prop(avgProp.diameter, sys.prop.angleRoot, sys.prop.angleTip, sys.prop.chordRoot, sys.prop.chordTip, 0, 0);
					break;
				case 6:
					counterProp = create_prop(sys.prop.diameter, avgProp.angleRoot, sys.prop.angleTip, sys.prop.chordRoot, sys.prop.chordTip, 0, 0);
					break;
				case 7:
					counterProp = create_prop(sys.prop.diameter, sys.prop.angleRoot, avgProp.angleTip, sys.prop.chordRoot, sys.prop.chordTip, 0, 0);
					break;
				case 8:
					counterProp = create_prop(sys.prop.diameter, sys.prop.angleRoot, sys.prop.angleTip, avgProp.chordRoot, sys.prop.chordTip, 0, 0);
					break;
				case 9:
					counterProp = create_prop(sys.prop.diameter, sys.prop.angleRoot, sys.prop.angleTip, sys.prop.chordRoot, avgProp.chordTip, 0, 0);
					break;
				case 10:
					counterRod = create_rod(avgRod.mat, sys.rod.Length, sys.rod.Dia, sys.rod.Thick);
					break;
				case 11:
					counterRod = create_rod(sys.rod.mat, sys.rod.Length, avgRod.Dia, sys.rod.Thick);
					break;
				case 12:
					counterRod = create_rod(sys.rod.mat, sys.rod.Length, sys.rod.Dia, avgRod.Thick);
					break;
				default:
					std::cout << "crust, too many agents" << std::endl;
			}
			switch (ag)
			{
				case 0: // or
				case 1: // or
				case 2: counterSys.battery = counterBattery; break;
				case 3: counterSys.motor = avgMotor; break;
				case 4: counterSys.foil = avgFoil; break;
				case 5: // or
				case 6: // or
				case 7: // or
				case 8: // or
				case 9: counterSys.prop = counterProp; break;
				case 10: // or
				case 11: // or
				case 12: counterSys.rod = counterRod; break;
			}
			rewards.ATD(ag, 0) = G - calc_G(penalty, counterSys);
		}
	}
	return rewards;
}

double calc_G(const sPenalty penalty, const sSys sys)
{
	double flightTime; cv::Mat constraints; sPerf hover; // these are all dummies
	return calc_G(penalty, sys, flightTime, constraints, hover);
}

double calc_G(const sPenalty penalty, const sSys sys, double & flightTime, cv::Mat & constraints, sPerf & hover)
{
	double G;
	
	write_propfile(sys.prop, sys.foil);

	int fail = 0;
	hover = calc_hover(sys);
	constraints = calc_constraints(sys, hover, fail);
	double totalCost = sys.battery.Cost + sys.motor.Cost*4 + sys.rod.Cost*4;
	flightTime = sys.battery.Energy/(4.0*hover.pelec.ATD(0, 0));

	if (hover.pelec.empty())
		fail = 1;
	else if (hover.failure == 1)
		fail = 1;

	G = flightTime; // default
	if (fail)
		G = penalty.failure;
	else
	{
		if (penalty.Mode == "death")
		{
			for (int i = 0; i < constraints.total(); i++)
			{
				if (constraints.ATD(i, 0) > 0)
				{
					G = penalty.death; // never mind, off with your head
					break;
				}
			}
		}
		else if (penalty.Mode == "deathplus")
		{
			int death = 0;
			cv::Mat conRewards = cv::Mat::zeros(constraints.size(), CV_64F);
			for (int i = 0; i < constraints.total(); i++)
			{
				if (constraints.ATD(i, 0) > 0)
				{
					death = 1;
					conRewards.ATD(i, 0) = penalty.lin*constraints.ATD(i, 0);
				}
			}
			if (death)
				G = penalty.death + cv::sum(conRewards)[0];
		}
		else if (penalty.Mode == "quad")
		{
			cv::Mat conRewards = cv::Mat::zeros(constraints.size(), CV_64F);
			for (int i = 0; i < constraints.total(); i++)
			{
				if (constraints.ATD(i, 0) > 0)
					conRewards.ATD(i, 0) = -penalty.R*pow(1 + constraints.ATD(i, 0), 2.0);
			}
			double temp = flightTime + cv::sum(conRewards)[0];
			if (penalty.quadtrunc >= temp)
				G = penalty.quadtrunc;
			else
				G = temp;
		}
		else if (penalty.Mode == "const")
		{
			cv::Mat conRewards = cv::Mat::zeros(constraints.size(), CV_64F);
			for (int i = 0; i < constraints.total(); i++)
			{
				if (constraints.ATD(i, 0) > 0)
					conRewards.ATD(i, 0) = -penalty.Const;
			}
			double temp = flightTime + cv::sum(conRewards)[0];
			if (penalty.quadtrunc >= temp)
				G = penalty.quadtrunc;
			else
				G = temp;
		}
		else if (penalty.Mode == "lin")
		{
			cv::Mat conRewards = cv::Mat::zeros(constraints.size(), CV_64F);
			for (int i = 0; i < constraints.total(); i++)
			{
				if (constraints.ATD(i, 0) > 0)
					conRewards.ATD(i, 0) = penalty.lin*constraints.ATD(i, 0) - 100;
			}
			G = flightTime + cv::sum(conRewards)[0];
		}
		else if (penalty.Mode == "none")
		{
			// Do Nothing
		}
		else
			std::cout << "SOMEONE SCREWED UP" << std::endl;
	}
}

sPerf calc_hover(const sSys sys)
{
	double thrustReq = sys.mass*9.81/4.0;

	std::string velStr = "0.0";
	std::string thrustStr = to_str<double>(thrustReq);
	std::string mode = "singlepoint";
	std::string rpmStr = "0";
	std::string voltStr = "0";
	std::string dBetaStr = "0";
	std::string torqueStr = "0";
	std::string ampsStr = "0";
	std::string peleStr = "0";

	sPerf hover = call_qprop(velStr, rpmStr, voltStr, dBetaStr, thrustStr, torqueStr, ampsStr, peleStr, mode, sys.motor.Num);

	if (hover.thrust.ATD(0,0) < 0.9*thrustReq)
		hover.failure = 1;
	else hover.failure = 0;

	return hover; // Can't exactly check for nan...
}

sPerf call_qprop(std::string velStr, std::string rpmStr, std::string voltStr, std::string dBetaStr, std::string thrustStr, 
				std::string torqueStr, std::string ampsStr, std::string peleStr, std::string mode, int motorNum)
{
	std::string motorNumStr = to_str<int>(motorNum);
	std::string qpropinput = "qprop propfile motorfiles\\motorfile" + motorNumStr + " " + velStr + " " + \
						rpmStr + " " + voltStr + " " + dBetaStr + " " + thrustStr + " " + torqueStr +\
						" " + ampsStr + " " + peleStr + "[\"]";

	FILE* qpropPipe = popen(qpropinput.c_str(), "r");
	char buffer[128];
    std::string result = "";
	if (qpropPipe)
	{ // Get output of program
        while (!feof(qpropPipe)) { // while not end of file
            if (fgets(buffer, sizeof(buffer), qpropPipe) != NULL)
                result += buffer; // Add contents of buffer to string
        }
	}
    pclose(qpropPipe);
	
	std::stringstream ss(result);
	std::string line;
	cv::Mat qpropoutput;
	int i = 0;
	/* read output line by line */
	if (mode == "multipoint")
	{
		while (std::getline(ss, line, '\n'))
		{
			if (line == " " || line == "\n" || line == "" || line.c_str()[0] == '#')
				continue;
			cv::Mat crayon = cv::Mat::zeros(1, 24, CV_64F);
			// TODO TODO TODO TODO TODO TODO TODO
			sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&crayon.ATD(0, 0), &crayon.ATD(0, 1), &crayon.ATD(0, 2), &crayon.ATD(0, 3), 
				&crayon.ATD(0, 4), &crayon.ATD(0, 5), &crayon.ATD(0, 6), &crayon.ATD(0, 7), 
				&crayon.ATD(0, 8), &crayon.ATD(0, 9), &crayon.ATD(0, 10), &crayon.ATD(0, 11), 
				&crayon.ATD(0, 12), &crayon.ATD(0, 13), &crayon.ATD(0, 14), &crayon.ATD(0, 15), 
				&crayon.ATD(0, 16), &crayon.ATD(0, 17), &crayon.ATD(0, 18), &crayon.ATD(0, 19), 
				&crayon.ATD(0, 20), &crayon.ATD(0, 21), &crayon.ATD(0, 22), &crayon.ATD(0, 23));

			assign_mat(qpropoutput, crayon.t(), i, i, -1, -1, -1, -1, -1, -1);
			i++;
		}
	}
	else // singlepoint
	{
		// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 
		while (std::getline(ss, line, '\n'))
		{
			if (line == " " || line == "\n" || line == "" || line.c_str()[0] == '#')
				continue;
			cv::Mat crayon = cv::Mat::zeros(1, 24, CV_64F);
			// TODO TODO TODO TODO TODO TODO TODO
			sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&crayon.ATD(0, 0), &crayon.ATD(0, 1), &crayon.ATD(0, 2), &crayon.ATD(0, 3), 
				&crayon.ATD(0, 4), &crayon.ATD(0, 5), &crayon.ATD(0, 6), &crayon.ATD(0, 7), 
				&crayon.ATD(0, 8), &crayon.ATD(0, 9), &crayon.ATD(0, 10), &crayon.ATD(0, 11), 
				&crayon.ATD(0, 12), &crayon.ATD(0, 13), &crayon.ATD(0, 14), &crayon.ATD(0, 15), 
				&crayon.ATD(0, 16), &crayon.ATD(0, 17), &crayon.ATD(0, 18), &crayon.ATD(0, 19), 
				&crayon.ATD(0, 20), &crayon.ATD(0, 21), &crayon.ATD(0, 22), &crayon.ATD(0, 23));

			assign_mat(qpropoutput, crayon.t(), i, i, -1, -1, -1, -1, -1, -1);
			i++;
			break; // ???
			// Or maybe we just have to read line 18
		}
	}
	sPerf perf;
	assign_mat(perf.velocity, qpropoutput, -1, -1, -1, -1, -1, -1, 0, 0);
	assign_mat(perf.rpm, qpropoutput, -1, -1, -1, -1, -1, -1, 1, 1);
	assign_mat(perf.dbeta, qpropoutput, -1, -1, -1, -1, -1, -1, 2, 2);
	assign_mat(perf.thrust, qpropoutput, -1, -1, -1, -1, -1, -1, 3, 3);
	assign_mat(perf.q, qpropoutput, -1, -1, -1, -1, -1, -1, 4, 4);
	assign_mat(perf.pshaft, qpropoutput, -1, -1, -1, -1, -1, -1, 5, 5);
	assign_mat(perf.volts, qpropoutput, -1, -1, -1, -1, -1, -1, 6, 6);
	assign_mat(perf.amps, qpropoutput, -1, -1, -1, -1, -1, -1, 7, 7);
	assign_mat(perf.effmotor, qpropoutput, -1, -1, -1, -1, -1, -1, 8, 8);
	assign_mat(perf.effprop, qpropoutput, -1, -1, -1, -1, -1, -1, 9, 9);
	assign_mat(perf.adv, qpropoutput, -1, -1, -1, -1, -1, -1, 10, 10);
	assign_mat(perf.ct, qpropoutput, -1, -1, -1, -1, -1, -1, 11, 11);
	assign_mat(perf.cp, qpropoutput, -1, -1, -1, -1, -1, -1, 12, 12);
	assign_mat(perf.dv, qpropoutput, -1, -1, -1, -1, -1, -1, 13, 13);
	assign_mat(perf.eff, qpropoutput, -1, -1, -1, -1, -1, -1, 14, 14);
	assign_mat(perf.pelec, qpropoutput, -1, -1, -1, -1, -1, -1, 15, 15);
	assign_mat(perf.pprop, qpropoutput, -1, -1, -1, -1, -1, -1, 16, 16);
	assign_mat(perf.clavg, qpropoutput, -1, -1, -1, -1, -1, -1, 17, 17);
	assign_mat(perf.cdavg, qpropoutput, -1, -1, -1, -1, -1, -1, 18, 18);
	return perf;
}

void run_experiment(sPenalty penalty, int numGens, int numRuns, int popSize, int D)
{
	sData data = load_data();

	double penFxnB = log(penalty.quadMin/penalty.quadMax)/(1 - numGens);
	double penFxnA = penalty.quadMin/exp(penFxnB);

	cv::Mat G_hist = cv::Mat::zeros(numRuns, numGens, CV_64F);
	cv::Mat flightTime_hist = cv::Mat::zeros(numRuns, numGens, CV_64F);
	
	// having trouble making 3d matrices...
	/*
	int sz[3] = {NUMAGENTS, numRuns, numGens};
	cv::Mat actions_hist = cv::Mat::zeros(3, sz, CV_64FC3);
	cv::Mat states_hist = cv::Mat::zeros(3, , CV_64FC3);
	*/

	
	/*
	cv::Mat constraint_hist = cv::Mat::zeros(3, sz, CV_64F);
	*/
	// perf_hist?
	// hover_hist?

	cv::Mat maxG = cv::Mat::zeros(numRuns, 1, CV_64F);
	cv::Mat genOfMax = cv::Mat::ones(numRuns, 1, CV_32S) * -1; // all -1
	//cv::Mat maxFlightTime = cv::Mat::zeros(numRuns, 1, CV_64F);

	cv::Mat actions = cv::Mat::zeros(NUMAGENTS, 1, CV_64F);
	sBattery battery;
	sMotor motor;
	sFoil foil;
	sProp prop;
	sRod rod;
	sSys sys;
	
	for (int r = 0; r < numRuns; r++)
	{
		maxG.ATD(r, 0) = 0;
		int numHidden = 10;

		/* Number of (inputs, hidden, outputs) for agents */
		int arrNumInputs[] = {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
		int arrNumHidden[] = {numHidden, numHidden, numHidden, numHidden, numHidden, numHidden, 
				numHidden, numHidden, numHidden, numHidden, numHidden, numHidden, numHidden};
		int arrNumOutputs[] = {6, 10, 6, 24, 8, 1, 1, 1, 1, 1, 4, 1, 1};

		std::vector<int> vecNumInputs(arrNumInputs, arrNumInputs + sizeof(arrNumInputs)/sizeof(int));
		std::vector<int> vecNumHidden(arrNumHidden, arrNumHidden + sizeof(arrNumHidden)/sizeof(int));
		std::vector<int> vecNumOutputs(arrNumOutputs, arrNumOutputs + sizeof(arrNumOutputs)/sizeof(int));
	
		CCEA ccea(NUMAGENTS, popSize, 1.0, 0.5);
		ccea.Init(vecNumInputs, vecNumHidden, vecNumOutputs);
		
		for (int g = 0; g < numGens; g++)
		{
			penalty.R = penFxnA*exp(penFxnB*g);
			
			ccea.Mutate();
			ccea.CreateTeams();

			double G;
			
			cv::Mat team_G = cv::Mat::zeros(popSize*2, 1, CV_64F);
			cv::Mat team_flightTime = cv::Mat::zeros(popSize*2, 1, CV_64F);
			// team_perf
			// team_hover
			cv::Mat team_constraints = cv::Mat::zeros(popSize*2, 8, CV_64F);
			cv::Mat team_rewards = cv::Mat::zeros(NUMAGENTS, popSize*2, CV_64F);
			for (int t = 0; t < popSize*2; t++)
			{
				//std::vector< std::vector<double> > (*GetOutputs)(const int) = ccea.GetOutputsOfTeam;
				
				assign_mat(actions, get_actions(t, &ccea));
				/*
				for (int ag = 0; ag < NUMAGENTS; ag++)
				{ // Can only access elements of 3D matrices with arrays
					int elem[] = {ag, r, g};
					actions_hist.at<double>(elem) = actions.ATD(ag, 0);
				}
				*/
				
				battery = design_battery(actions, data.batteryData);
				motor = design_motor(actions, data.motorData);
				prop = design_prop(actions, data.foilData);
				foil = design_foil(actions, data.foilData);
				rod = design_rod(actions, data.matData, prop);
				sys = design_sys(battery, motor, foil, prop, rod);

				cv::Mat constraints;
				double flightTime;
				sPerf perf, hover;
				cv::Mat rewards = compute_rewards(D, penalty, sys, data, /* <-- inputs */
								G, flightTime, constraints, hover); /* <-- outputs */
				team_G.ATD(t, 0) = G;
				team_flightTime.ATD(t, 0) = flightTime;
				assign_mat(team_constraints, constraints, t, t, -1, -1, -1, -1, -1, -1);
				assign_mat(team_rewards, rewards, -1, -1, t, t, -1, -1, -1, -1);
				
				ccea.AssignFitness(t, Mat_to_vector_double(rewards));
				
				update_states(ccea.statesPtrs[t], hover, constraints, sys);
			}

			int bestTeam;
			G = max_double(team_G, bestTeam);
			ccea.ReformPopulations();
			ccea.SelectFittest();
			/*
			for (int c = 0; c < 8; c++)
			{
				int elem[] = {c, r, g};
				constraint_hist.at<double>(elem) = team_constraints.ATD(bestTeam, c);
			}
			*/

			G_hist.ATD(r, g) = G;
			flightTime_hist.ATD(r, g) = team_flightTime.ATD(bestTeam, 0);
			
			cv::Mat temp = cv::Mat::ones(1, 1, CV_32S)*bestTeam;
			if (G > maxG.ATD(r, 0) && all_leqz(mat_from_indices(team_constraints, temp, cv::Mat())));
			{
				maxG.ATD(r, 0) = G;
				genOfMax.at<int>(r, 0) = g;
				//maxFlightTime.ATD(r, 0) = team_flightTime.ATD(bestTeam, 0);
			}
			
			std::cout << r << ", " << g << std::endl;
		}
	}
	double avgG = cv::mean(G_hist)[0];

	/* [record juicy info here] */
}

// COUNTER CALC STUFF
	/*
	cv::Mat empty;
	avgCell.Cost = mean_double(mat_from_indices(data.batteryData, empty, cv::Mat::ones(1, 1, CV_32S)*0));
	avgCell.Cap = mean_double(mat_from_indices(data.batteryData, empty, cv::Mat::ones(1, 1, CV_32S)*1))/1000.0;
	avgCell.C = mean_double(mat_from_indices(data.batteryData, empty, cv::Mat::ones(1, 1, CV_32S)*2));
	avgCell.Mass = mean_double(mat_from_indices(data.batteryData, empty, cv::Mat::ones(1, 1, CV_32S)*3))/1000.0;
	avgCell.Length = mean_double(mat_from_indices(data.batteryData, empty, cv::Mat::ones(1, 1, CV_32S)*4));
	avgCell.Width = mean_double(mat_from_indices(data.batteryData, empty, cv::Mat::ones(1, 1, CV_32S)*5));
	avgCell.Height = mean_double(mat_from_indices(data.batteryData, empty, cv::Mat::ones(1, 1, CV_32S)*6));

	avgMotor.kv = mean_double(mat_from_indices(data.motorData, empty, cv::Mat::ones(1, 1, CV_32S)*0));
	avgMotor.R0 = mean_double(mat_from_indices(data.motorData, empty, cv::Mat::ones(1, 1, CV_32S)*1))/1000.0;
	avgMotor.I0 = mean_double(mat_from_indices(data.motorData, empty, cv::Mat::ones(1, 1, CV_32S)*2));
	avgMotor.Imax = mean_double(mat_from_indices(data.motorData, empty, cv::Mat::ones(1, 1, CV_32S)*3));
	avgMotor.Pmax = mean_double(mat_from_indices(data.motorData, empty, cv::Mat::ones(1, 1, CV_32S)*4));
	avgMotor.Mass = mean_double(mat_from_indices(data.motorData, empty, cv::Mat::ones(1, 1, CV_32S)*5))/1000.0;
	avgMotor.Cost = mean_double(mat_from_indices(data.motorData, empty, cv::Mat::ones(1, 1, CV_32S)*6));
	avgMotor.Diam = mean_double(mat_from_indices(data.motorData, empty, cv::Mat::ones(1, 1, CV_32S)*7))/1000.0;
	avgMotor.planArea = (M_PI/4.0)*pow(avgMotor.Diam, 2.0);
	avgMotor.Num = -1;

	avgProp.diameter = mean_double(mat_from_indices(data.propData, empty, cv::Mat::ones(1, 1, CV_32S)*0));
	avgProp.angleRoot = mean_double(mat_from_indices(data.propData, empty, cv::Mat::ones(1, 1, CV_32S)*1));
	avgProp.angleTip = mean_double(mat_from_indices(data.propData, empty, cv::Mat::ones(1, 1, CV_32S)*2));
	avgProp.chordRoot = mean_double(mat_from_indices(data.propData, empty, cv::Mat::ones(1, 1, CV_32S)*3));
	avgProp.chordTip = mean_double(mat_from_indices(data.propData, empty, cv::Mat::ones(1, 1, CV_32S)*4));
	
	avgFoil.Cl0 = mean_double(mat_from_indices(data.foilData, empty, cv::Mat::ones(1, 1, CV_32S)*0));
	avgFoil.Cla = mean_double(mat_from_indices(data.foilData, empty, cv::Mat::ones(1, 1, CV_32S)*1)*360/(2*M_PI));
	avgFoil.Clmin = mean_double(mat_from_indices(data.foilData, empty, cv::Mat::ones(1, 1, CV_32S)*2));
	avgFoil.Clmax = mean_double(mat_from_indices(data.foilData, empty, cv::Mat::ones(1, 1, CV_32S)*3));
	avgFoil.Cd0 = mean_double(mat_from_indices(data.foilData, empty, cv::Mat::ones(1, 1, CV_32S)*4));
	avgFoil.Cd2 = mean_double(mat_from_indices(data.foilData, empty, cv::Mat::ones(1, 1, CV_32S)*5)*360/(2*M_PI));
	avgFoil.Clcd0 = mean_double(mat_from_indices(data.foilData, empty, cv::Mat::ones(1, 1, CV_32S)*6));
	avgFoil.Reref = mean_double(mat_from_indices(data.foilData, empty, cv::Mat::ones(1, 1, CV_32S)*7));
	avgFoil.Reexp = mean_double(mat_from_indices(data.foilData, empty, cv::Mat::ones(1, 1, CV_32S)*8));
	avgFoil.Num = (int)round(mean_double(mat_from_indices(data.foilData, empty, cv::Mat::ones(1, 1, CV_32S)*9)));

	avgMat.Ymod = mean_double(mat_from_indices(data.matData, empty, cv::Mat::ones(1, 1, CV_32S)*0));
	avgMat.Sut = mean_double(mat_from_indices(data.matData, empty, cv::Mat::ones(1, 1, CV_32S)*1));
	avgMat.Sy = mean_double(mat_from_indices(data.matData, empty, cv::Mat::ones(1, 1, CV_32S)*2));
	avgMat.Dens = mean_double(mat_from_indices(data.matData, empty, cv::Mat::ones(1, 1, CV_32S)*3));
	avgMat.Cost = mean_double(mat_from_indices(data.matData, empty, cv::Mat::ones(1, 1, CV_32S)*4))*pow(100/2.54, 3.0);

	avgRod.mat = avgMat;
	avgRod.Length = mean_double(mat_from_indices(data.matData, empty, cv::Mat::ones(1, 1, CV_32S)*0));
	*/
