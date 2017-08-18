/*
 * RINEXnav.cpp
 *
 *  Created on: 13 Aug 2017
 *      Author: Sokol
 */
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <math.h>

#include "RINEXnav.h"
#include "navmsg.h"
using namespace std;

// todo: add const string& strFormat
double stoDouble(const string& strValue) {
	// defined for format: D19.12
	double Value = stod(strValue.substr(0, 15));
	double Power = stod(strValue.substr(16, 3));
	Value *= pow(10, Power);
	return Value;
}

glonass_nav_msg& getGLONASSnavmsgBlock(ifstream& textfile,
		glonass_nav_msg& glo_nav_msg) {
	string str_line;
	glonass_nav_data_block glo_nav_data;

	getline(textfile, str_line);
	int satNumber = stoi(str_line.substr(1, 2));
	glo_nav_msg.sats.push_back(satNumber);

	Epoch epoch;
	epoch.year = stoi(str_line.substr(4, 4));
	epoch.month = stoi(str_line.substr(9, 2));
	epoch.day = stoi(str_line.substr(12, 2));
	epoch.hour = stoi(str_line.substr(15, 2));
	epoch.minutes = stoi(str_line.substr(18, 2));
	epoch.seconds = stof(str_line.substr(21, 2));

	glo_nav_data.epoch = epoch;
	glo_nav_data.SVClockBias = stoDouble(str_line.substr(23, 19));
	glo_nav_data.SVRelativeFrequencyBias = stoDouble(str_line.substr(42, 19));
	glo_nav_data.MessageFrameTime = stoDouble(str_line.substr(61, 19));

	// convert [km] [km/s] [km/s^2] to [m], [m/s], [m/s^2]
	getline(textfile, str_line);
	glo_nav_data.x = stoDouble(str_line.substr(4, 19)) * 1000;
	glo_nav_data.vx = stoDouble(str_line.substr(23, 19)) * 1000;
	glo_nav_data.ax = stoDouble(str_line.substr(42, 19)) * 1000;
	glo_nav_data.SatelliteHealth = stoDouble(str_line.substr(61, 19));

	getline(textfile, str_line);
	glo_nav_data.y = stoDouble(str_line.substr(4, 19)) * 1000;
	glo_nav_data.vy = stoDouble(str_line.substr(23, 19)) * 1000;
	glo_nav_data.ay = stoDouble(str_line.substr(42, 19)) * 1000;
	glo_nav_data.FrequencyNumber = stoi(str_line.substr(61, 19));

	getline(textfile, str_line);
	glo_nav_data.z = stoDouble(str_line.substr(4, 19)) * 1000;
	glo_nav_data.vz = stoDouble(str_line.substr(23, 19)) * 1000;
	glo_nav_data.az = stoDouble(str_line.substr(42, 19)) * 1000;
	glo_nav_data.InformationAge = stoDouble(str_line.substr(61, 19));

	glo_nav_msg.nav[satNumber] = glo_nav_data;

	return glo_nav_msg;
}

glonass_nav_msg parse_GLONASS_Nav(ifstream& textfile) {

	glonass_nav_msg glo_nav_msg;
	glo_nav_msg.system = "GLONASS";
	// parse header
	string str_line;
	while (getline(textfile, str_line)) {
		if (str_line.substr(60, 20) == "TIME SYSTEM CORR") {
			break;
		}
	}
	glo_nav_msg.header.SystemCorrectiontTme = stod(str_line.substr(6, 16));
	getline(textfile, str_line);
	glo_nav_msg.header.LeapSeconds = stoi(str_line.substr(0, 6));

	// skip rest of header
	while (getline(textfile, str_line)) {
		if (str_line.substr(60, 20) == "END OF HEADER") {
			break;
		}
	}

	for (int i = 0; i < 24; ++i) {
		glo_nav_msg = getGLONASSnavmsgBlock(textfile, glo_nav_msg);
	}

	sort(glo_nav_msg.sats.begin(), glo_nav_msg.sats.end());
	return glo_nav_msg;
}

glonass_nav_msg parseRINEX_Nav(ifstream& nav_file_stream) {
	string str_line;
	getline(nav_file_stream, str_line);
	if (str_line.substr(20, 1) != "N") {
		cout << "Error, is not navigation file: " << endl;
		throw "Error, is not navigation file";
	}

	glonass_nav_msg glo_nav_msg;
	if (str_line.substr(40, 1) == "R") {
		glo_nav_msg = parse_GLONASS_Nav(nav_file_stream);
		return glo_nav_msg;
	} else {
		cout << "Error, other systems are not implemented" << endl;
		throw "Error, other systems are not implemented";
	}
}

void printGLOnavmsg(const glonass_nav_msg& glo_msg) {
	cout << glo_msg.system << endl;
//	cout << "LeapSeconds " <<  glo_msg.header.LeapSeconds << endl;
//	cout << "SystemCorrectiontTme " <<  glo_msg.header.SystemCorrectiontTme << endl;
	for (auto& navBlock : glo_msg.nav) {
		int SV = navBlock.first;
		cout << "SV " << setw(2) << SV << " : xyz : " << fixed << setprecision(6) << setw(20)
				<< navBlock.second.x << " " << setw(20)
				<< navBlock.second.y << " " << setw(20)
				<< navBlock.second.z << "   [m] " << navBlock.second.epoch.minutes << endl;
	}
}