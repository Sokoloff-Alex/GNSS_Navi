/*
 * navmsg.h
 *
 *  Created on: 12 Aug 2017
 *      Author: Sokol
 */

#ifndef NAVMSG_H_
#define NAVMSG_H_

#include <string>
#include <map>
#include <vector>
#include <set>
#include "Matrix.h"

using namespace std;

struct glonass_nav_header {
	double SystemCorrectiontTme; // TauC
	int LeapSeconds;
};

class Epoch {
public:
	int year;
	int month;
	int day;
	int hour;
	int minutes;
	double seconds;

//	Epoch(const int& myYear, const int& myMonth, const int& myDay, const int& myHour, const int& myMinute, const double& mySecond);
	double toSeconds() const;
	Epoch addSec(const double& sec);
	bool operator<(const Epoch& rhs);
	bool operator==(const Epoch& rhs);
};

class Orbits {
public:
	Epoch epoch;
	map<int, Matrix > StateVector;
	map<int, Matrix > Atb;
	map<int, double> SVClockBias;
	map<int, double> SVRelativeFrequencyBias;
	map<int, int> FrequencyNumber;
	vector<int> sats;
	double SystemCorrectiontTme; // TauC

};

struct glonass_nav_data_block {
	int slotNumber;
	Epoch epoch;
	double SVClockBias;
	double SVRelativeFrequencyBias;
	double MessageFrameTime;
	double SatelliteHealth;
	int FrequencyNumber;
	double InformationAge;
	double x;
	double y;
	double z;
	double vx;
	double vy;
	double vz;
	double ax;
	double ay;
	double az;
};

struct glonass_nav_msg {
	string system;
	glonass_nav_header header;
	map<int, glonass_nav_data_block> nav;
	vector<int> sats;
	string stringBuffer;
};

Matrix getStaveVectorXV(const glonass_nav_data_block& msgSV);

Matrix getStaveVectorX(const glonass_nav_data_block& msgSV);
Matrix getStaveVectorV(const glonass_nav_data_block& msgSV);
Matrix getStaveVectorA(const glonass_nav_data_block& msgSV);


Orbits select(const Orbits& orbs, const set<int>& sats);

void printSet(const set<int>& s);

#endif /* NAVMSG_H_ */
