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

void parseGLONASSnavmsgBlock(ifstream& textfile, glonass_nav_msg& glo_nav_msg) {
	string str_line;
	getline(textfile, str_line);
	glonass_nav_data_block glo_nav_data;
	fillNavBlock_head(str_line, glo_nav_data);
	fillNavBlock_tail(textfile, glo_nav_data);

	glo_nav_msg.nav[glo_nav_data.slotNumber] = glo_nav_data;
	glo_nav_msg.sats.push_back(glo_nav_data.slotNumber);
}

void fillNavBlock_head(const string& firstLineOfBlock, glonass_nav_data_block& glo_nav_data) {

	glo_nav_data.slotNumber = stoi(firstLineOfBlock.substr(1, 2));
	glo_nav_data.epoch = parseNavEpoch(firstLineOfBlock);
	glo_nav_data.SVClockBias = stoDouble(firstLineOfBlock.substr(23, 19));
	glo_nav_data.SVRelativeFrequencyBias = stoDouble(firstLineOfBlock.substr(42, 19));
	glo_nav_data.MessageFrameTime = stoDouble(firstLineOfBlock.substr(61, 19));
}

void fillNavBlock_tail(ifstream& textfile, glonass_nav_data_block& glo_nav_data) {

	string str_line;
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

	glo_nav_msg.header.SystemCorrectiontTme = stoDouble(str_line.substr(5, 17),
			"D17.10");
	getline(textfile, str_line);
	glo_nav_msg.header.LeapSeconds = stoi(str_line.substr(0, 6));

	// skip rest of header
	while (getline(textfile, str_line)) {
		if (str_line.substr(60, 20) == "END OF HEADER") {
			break;
		}
	}

	for (int i = 0; i < 24; ++i) {
		parseGLONASSnavmsgBlock(textfile, glo_nav_msg);
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
		getline(nav_file_stream, str_line);
		glo_nav_msg.stringBuffer = str_line;
		return glo_nav_msg;
	} else {
		cout << "Error, other systems are not implemented" << endl;
		throw "Error, other systems are not implemented";
	}
}

void updateMSGfromRINEX(ifstream& nav_file_stream, glonass_nav_msg& glo_msg) {

	glonass_nav_data_block glo_nav_data;

//	update from First incoming block
	string str_line = glo_msg.stringBuffer;
	Epoch epoch_block_first = parseNavEpoch(str_line);
	fillNavBlock_head(str_line, glo_nav_data);
	fillNavBlock_tail(nav_file_stream, glo_nav_data);
	glo_msg.nav[glo_nav_data.slotNumber] = glo_nav_data;

//	try to update from next incoming block
	getline(nav_file_stream, str_line);
	Epoch epoch_block_next = parseNavEpoch(str_line);
	while (epoch_block_first == epoch_block_next) {
		fillNavBlock_head(str_line, glo_nav_data);
		fillNavBlock_tail(nav_file_stream, glo_nav_data);
		glo_msg.nav[glo_nav_data.slotNumber] = glo_nav_data;

		// if ok, try another
		getline(nav_file_stream, str_line);
		epoch_block_next = parseNavEpoch(str_line);
	}
	glo_msg.stringBuffer = str_line;
}

void printGLOnavmsg(const glonass_nav_msg& glo_msg) {
	cout << glo_msg.system << endl;
//	cout << "LeapSeconds " <<  glo_msg.header.LeapSeconds << endl;
//	cout << "SystemCorrectiontTme " <<  glo_msg.header.SystemCorrectiontTme << endl;
	for (auto& navBlock : glo_msg.nav) {
		int SV = navBlock.first;
		cout << "SV " << setw(2) << SV << " : xyz : " << fixed
				<< setprecision(6) << setw(20) << navBlock.second.x << " "
				<< setw(20) << navBlock.second.y << " " << setw(20)
				<< navBlock.second.z << "   [m] "
				<< navBlock.second.epoch.minutes << endl;
	}
}

Epoch parseNavEpoch(const string& str_line) {
	Epoch epoch;
	epoch.year = stoi(str_line.substr(4, 4));
	epoch.month = stoi(str_line.substr(9, 2));
	epoch.day = stoi(str_line.substr(12, 2));
	epoch.hour = stoi(str_line.substr(15, 2));
	epoch.minutes = stoi(str_line.substr(18, 2));
	epoch.seconds = stod(str_line.substr(21, 2));
	return epoch;
}

double stoDouble(const string& strValue, const string& strFormat) {
//	 example: D17.10	" 1.5832483768D-08"
	int LenTotal = stoi(strFormat.substr(1, 2));
	int LenValue = LenTotal - 4;
	double Value = stod(strValue.substr(0, LenValue));
	double Power = stod(strValue.substr(LenValue + 1, 3));
	Value *= pow(10, Power);
	return Value;
}

double stoDouble(const string& strValue) {
	// defined for format: D19.12
	double Value = stod(strValue.substr(0, 15));
	double Power = stod(strValue.substr(16, 3));
	Value *= pow(10, Power);
	return Value;
}

