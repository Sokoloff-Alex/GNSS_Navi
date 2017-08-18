/*
 * navmsg.cpp
 *
 *  Created on: 12 Aug 2017
 *      Author: Sokol
 */

#include <set>
#include "navmsg.h"

double Epoch::toSeconds() const {
	double timeSeconds = hour * 3600 + minutes * 60 + seconds;
	return timeSeconds;
}

Matrix<double> getStaveVectorXV(const glonass_nav_data_block& msgSV) {

	Matrix<double> StateVector(6, 1, 0);
	StateVector(0) = msgSV.x;
	StateVector(1) = msgSV.y;
	StateVector(2) = msgSV.z;
	StateVector(3) = msgSV.vx;
	StateVector(4) = msgSV.vy;
	StateVector(5) = msgSV.vz;
	return StateVector;
}

Matrix<double> getStaveVectorA(const glonass_nav_data_block& msgSV) {

	Matrix<double> StateVector(3, 1, 0);
	StateVector(0, 0) = msgSV.ax;
	StateVector(1, 0) = msgSV.ay;
	StateVector(2, 0) = msgSV.az;
	return StateVector;
}

Orbits select(const Orbits& orbs, const set<int>& sats) {
	Orbits orbs_selected;
	map<int, Matrix<double> > StateVectors;
	for (int sat : sats) {
		StateVectors[sat] = orbs.StateVector.at(sat);
		orbs_selected.sats.push_back(sat);
	}
	orbs_selected.StateVector = StateVectors;
	orbs_selected.epoch = orbs.epoch;
	return orbs_selected;
}

Epoch Epoch::addSec(const int& sec) {
	seconds += sec;
	if ( (seconds / 60) > 1 ) {
		seconds -= 60;
		minutes++;
		if (!(minutes/60)) {
			minutes -= 60;
			hour++;
			if (!(hour/24)) {
				hour -= 60;
				day++;
			}
		}
	}
	return *this;
}
