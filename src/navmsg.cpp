/*
 * navmsg.cpp
 *
 *  Created on: 12 Aug 2017
 *      Author: Sokol
 */

#include <set>
#include "navmsg.h"

//Epoch::Epoch(const int& myYear = 1970, const int& myMonth = 1,
//		const int& myDay = 1, const int& myHour = 0, const int& myMinute = 0,
//		const double& mySecond = 0) {
//	year = myYear;
//	month = myMonth;
//	day = myDay;
//	hour = myHour;
//	minutes = myMinute;
//	seconds = mySecond;
//}

Epoch Epoch::addSec(const double& sec) {

	double timeTotalSec = this->toSeconds() + sec;

	seconds = fmod(timeTotalSec, 60);
	minutes = (int)(timeTotalSec - seconds) % (60 * 60);
	hour = (int)(timeTotalSec - seconds - (minutes * 60)) % (60 * 60 * 24);

	if (hour >= 24) {
		hour -= 24;
		day++;
	}
	return *this;
}

double Epoch::toSeconds() const {
	double timeSeconds = hour * 3600 + minutes * 60 + seconds;
	return timeSeconds;
}

bool Epoch::operator<(const Epoch& rhs) {
	if (this->year == rhs.year) {
		if (this->month == rhs.month) {
			if (this->day == rhs.day) {
				return this->toSeconds() < rhs.toSeconds();
			} else {
				return this->day < rhs.day;
			}
		} else {
			return this->month < rhs.month;
		}
	} else {
		return this->year < rhs.year;
	}
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
	for (int sat : sats) {
		orbs_selected.StateVector[sat] = orbs.StateVector.at(sat);
		orbs_selected.FrequencyNumber[sat] = orbs.FrequencyNumber.at(sat);
		orbs_selected.SVClockBias[sat] = orbs.SVClockBias.at(sat);
		orbs_selected.SVRelativeFrequencyBias[sat] =
				orbs.SVRelativeFrequencyBias.at(sat);
		orbs_selected.Atb[sat] = orbs.Atb.at(sat);
		orbs_selected.sats.push_back(sat);
	}
	orbs_selected.SystemCorrectiontTme = orbs.SystemCorrectiontTme;
	orbs_selected.epoch = orbs.epoch;
	return orbs_selected;
}

