/*
 * RINEX.cpp
 *
 *  Created on: 12 Aug 2017
 *      Author: Sokol
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

#include "Matrix.h"
#include "RINEX.h"
#include "navmsg.h"
#include "Observations.h"

using namespace std;

Observations parseRINEX_Epoch(ifstream& textfile) {
	string str;
	int NumberOfSatellitesTotal;
	Epoch epoch;
	vector<double> RangesStack;
	vector<string> SatelliteNamesStack;
	while (getline(textfile, str)) {
		if (str.substr(0, 1) != ">") {
			cout << "End of file" << endl;
			break;
		}
//		cout << str << endl;

		epoch = parseObsEpoch(str);
		NumberOfSatellitesTotal = stoi(str.substr(33, 3));
		SatelliteNamesStack.resize(NumberOfSatellitesTotal);
		RangesStack.resize(NumberOfSatellitesTotal);
		for (int i = 0; i < NumberOfSatellitesTotal; ++i) {
			if (getline(textfile, str)) {
				SatelliteNamesStack[i] = str.substr(0, 3);
				RangesStack[i] = stod(str.substr(5, 14));
			} else {
				cout << "Error, unexpected end of file" << endl;
				break;
			}
		}
		break;
	}
	// pack ranges into map (split by system)
	// <system, <satellite, range> >
	map<string, map<int, double> > Ranges;
	string system;
	int satellite;
	Observations Obs;
	for (int i = 0; i < NumberOfSatellitesTotal; ++i) {
		system = SatelliteNamesStack[i].substr(0, 1);
		satellite = stoi(SatelliteNamesStack[i].substr(1, 2));
		Obs.sats[system].push_back(satellite);
		Ranges[system][satellite] = RangesStack[i];
	}
	Obs.epoch = epoch;
	Obs.ranges = Ranges;
	return Obs;
}

Observations parseRINEX(ifstream& textfile) {

	// skip header
	string str;
	while (getline(textfile, str)) {
		if (str.substr(60, 20) == "END OF HEADER") {
			break;
		}
	}

	// get observations
	Observations obs = parseRINEX_Epoch(textfile);

	return obs;
}

Epoch parseObsEpoch(const string& str) {
	Epoch epoch;
	epoch.year = stoi(str.substr(2, 4));
	epoch.month = stoi(str.substr(7, 2));
	epoch.day= stoi(str.substr(10, 2));
	epoch.hour = stoi(str.substr(13, 2));
	epoch.minutes = stoi(str.substr(16, 2));
	epoch.seconds = stof(str.substr(19, 10));
	return epoch;
}
