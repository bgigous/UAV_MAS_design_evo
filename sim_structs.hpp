#ifndef SIM_STRUCTS_HPP
#define SIM_STRUCTS_HPP

#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp" 
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

/*
typedef struct sHistory
{
	Mat G_hist;
	Mat flightTime_hist;
	Mat actions;
	Mat states;
	Mat constraint;
	Mat perf; // tricky
	Mat hover; // tricky
	Mat
}
*/

typedef struct sPenalty
{
	std::string Mode;
	double quadMin;
	double quadMax;
	double quadtrunc;
	double Const;
	double death;
	double lin;
	double failure;
	double R;
} sPenalty;

typedef struct sData
{
	cv::Mat batteryData;
	cv::Mat motorData;
	cv::Mat propData;
	cv::Mat foilData;
	cv::Mat rodData;
	cv::Mat matData;
} sData;

typedef struct sPerf
{
	cv::Mat velocity;
	cv::Mat rpm;
	cv::Mat dbeta;
	cv::Mat thrust;
	cv::Mat q;
	cv::Mat pshaft;
	cv::Mat volts;
	cv::Mat amps;
	cv::Mat effmotor;
	cv::Mat effprop;
	cv::Mat adv;
	cv::Mat ct;
	cv::Mat cp;
	cv::Mat dv;
	cv::Mat eff;
	cv::Mat pelec;
	cv::Mat pprop;
	cv::Mat clavg;
	cv::Mat cdavg;
	int failure;
} sPerf;

typedef struct sHover
{
	double velocity;
	double rpm;
	double dbeta;
	double thrust;
	double q;
	double pshaft;
	double volts;
	double amps;
	double effmotor;
	double effprop;
	double adv;
	double ct;
	double cp;
	double dv;
	double eff;
	double pelec;
	double pprop;
	double clavg;
	double cdavg;
	int failure;
} sHover;

typedef struct sCell
{
	double Cost;
	double Cap;
	double C;
	double Mass;
} sCell;

typedef struct sBattery
{
	sCell Cell;
	int sConfigs;
	int pConfigs;
	double Cost;
	double Mass;
	double Volt;
	double Cap;
	double C;
	double Imax;
	double Energy;
} sBattery;

typedef struct sMotor
{
	double kv;
	double R0;
	double I0;
	double Imax;
	double Pmax;
	double Mass;
	double Cost;
	double Diam;
	double planArea;
	int Num;
} sMotor;

typedef struct sFoil
{
	double Cl0;
	double Cla;
	double Clmin;
	double Clmax;
	double Cd0;
	double Cd2;
	double Clcd0;
	double Reref;
	double Reexp;
	int Num;
} sFoil;

typedef struct sProp
{
	double diameter;
	double angleRoot;
	double angleTip;
	double chordRoot;
	double chordTip;
	double mass;
	double cost;
} sProp;

typedef struct sMaterial
{
	int Type;
	double Ymod;
	double Sut;
	double Sy;
	double Dens;
	double Cost;
} sMaterial;

typedef struct sRod
{
	sMaterial mat;
	double Length;
	double Dia;
	double Thick;
	double Area;
	double Amoment;
	double Stiffness;
	double Vol;
	double Mass;
	double Cost;
	double planArea;
} sRod;

typedef struct sRes
{
	double mass;
	double framewidth;
	double planArea;
	double cost;
	double power;
} sRes;

typedef struct sSys
{
	sBattery battery;
	sMotor motor;
	sFoil foil;
	sProp prop;
	sRod rod;
	sRes res;
	double mass;
	double planArea;
	double natFreq;
	double cost;
	double power;
} sSys;

#endif
