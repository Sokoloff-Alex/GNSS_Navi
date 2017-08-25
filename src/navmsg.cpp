/*
 * navmsg.cpp
 *
 *  Created on: 12 Aug 2017
 *      Author: Sokol
 */

#include <set>
#include <math.h>
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

bool Epoch::operator==(const Epoch& rhs) {
	if (this->year == rhs.year) {
		if (this->month == rhs.month) {
			if (this->day == rhs.day) {
				return (this->toSeconds() == rhs.toSeconds());
			} else {
				return this->day == rhs.day;
			}
		} else {
			return this->month == rhs.month;
		}
	} else {
		return this->year == rhs.year;
	}
}

Matrix getStaveVectorXV(const glonass_nav_data_block& msgSV) {

	Matrix StateVectorXV(6, 1, 0);
	StateVectorXV(0) = msgSV.x;
	StateVectorXV(1) = msgSV.y;
	StateVectorXV(2) = msgSV.z;
	StateVectorXV(3) = msgSV.vx;
	StateVectorXV(4) = msgSV.vy;
	StateVectorXV(5) = msgSV.vz;
	return StateVectorXV;
}

Matrix getStaveVectorX(const glonass_nav_data_block& msgSV) {

	Matrix StateVectorX(3, 1, 0);
	StateVectorX(0) = msgSV.x;
	StateVectorX(1) = msgSV.y;
	StateVectorX(2) = msgSV.z;
	return StateVectorX;
}

Matrix getStaveVectorV(const glonass_nav_data_block& msgSV) {

	Matrix StateVectorV(3, 1, 0);
	StateVectorV(0) = msgSV.vx;
	StateVectorV(1) = msgSV.vy;
	StateVectorV(2) = msgSV.vz;
	return StateVectorV;
}

Matrix getStaveVectorA(const glonass_nav_data_block& msgSV) {

	Matrix StateVectorA(3, 1, 0);
	StateVectorA(0, 0) = msgSV.ax;
	StateVectorA(1, 0) = msgSV.ay;
	StateVectorA(2, 0) = msgSV.az;
	return StateVectorA;
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

