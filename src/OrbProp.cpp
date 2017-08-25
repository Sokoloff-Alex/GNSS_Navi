/* OrbProp.cpp
 *
 * Created on: 14 Aug 2017
 * Author: Sokol
 */

#include "OrbProp.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include "Matrix.h"
#include "RINEXnav.h"
#include "boost/math/constants/constants.hpp"

using namespace std;

//find first time derivatives of state vector (oblate Earth model)
Matrix getDerivatives(const double& t, const Matrix& StateVector,
		const Matrix& Ams) {

	const double GM = 398600441.8 * pow(10, 6); // geocentric constant of Earth Gravity field (including Atmosphere), [m^3 / s^2]
	const double Ae = 6378136; // large (equatorial) semi-axis of ellipsoid, [m]
	const double J02 = 1082625.75 * pow(10, -9); // zonal harmonic coeff. of 2nd order (oblatness) [dimentionless]
	const double We = 7.2921151467 * pow(10, -5); // mean Earth rotation Rate, w.r.t. vernal equinox, [rad/s]

	double x = StateVector(0);
	double y = StateVector(1);
	double z = StateVector(2);
	double Vx = StateVector(3);
	double Vy = StateVector(4);

	double r2 = (x * x + y * y + z * z);
	double r = pow(r2, 0.5);

	double GM_by_r3 = GM / (r2 * r);

	double coeff2 = 3 / 2 * J02 * GM_by_r3 * Ae * Ae / r2;

	double z2_by_r2 = z * z / r2;

	Matrix StateVecDot(6, 1, 0.0);

	StateVecDot(0) = StateVector(3);
	StateVecDot(1) = StateVector(4);
	StateVecDot(2) = StateVector(5);

	StateVecDot(3) = -GM_by_r3 * x - coeff2 * x * (1 - 3 * z2_by_r2)
			+ We * We * x + 2 * We * Vy + Ams(0);
	StateVecDot(4) = -GM_by_r3 * y - coeff2 * y * (1 - 3 * z2_by_r2)
			+ We * We * y - 2 * We * Vx + Ams(1);
	StateVecDot(5) = -GM_by_r3 * z - coeff2 * z * (3 - 5 * z2_by_r2) + Ams(2);

	return StateVecDot;
}

Matrix RungeKutta4order(const Matrix& Y, const double& t, const double& h,
		const Matrix& Atb) {

	Matrix k1 = getDerivatives(t, Y, Atb) * h;
	Matrix k2 = getDerivatives(t + h / 2, k1 * h / 2 + Y, Atb);
	Matrix k3 = getDerivatives(t + h / 2, k2 * h / 2 + Y, Atb);
	Matrix k4 = getDerivatives(t + h, k3 * h + Y, Atb);

	Matrix Ynew = (k1 + k2 * 2 + k3 * 2 + k4) / 6 + Y;
	return Ynew;
}

Orbits propagateOrbits(const glonass_nav_msg& glo_msg,
		const Epoch& TargetEpoch) {

	ofstream file_orb("D:/Dev/GNSS/obs/orbits.txt");
	vector<int> sats;
	Orbits LatestStatesAll;
	Matrix StateVector(6, 1, 0.0);
	Matrix A_tb(3, 1, 0.0);
	Matrix A_tb_dummy(6, 1, 0.0);
	for (auto& msgSV : glo_msg.nav) {
		if (TargetEpoch.hour == 0 && TargetEpoch.minutes <= 15) {
			double stepT = 1;
			double T0 = msgSV.second.epoch.toSeconds();
			double Ttarget = TargetEpoch.toSeconds();
//			if (EphemerisEpoch.hour == 23 && EphemerisEpoch.minutes >= 45) { // propagate only recent satellites
//				T0 = -15 * 60; // last 15 minutes only, keep negative, [sec]
//			}
			int SV = msgSV.first;
			StateVector = getStaveVectorXV(msgSV.second);
			A_tb = getStaveVectorA(msgSV.second);
			A_tb_dummy(0) = A_tb(0);
			A_tb_dummy(1) = A_tb(1);
			A_tb_dummy(2) = A_tb(2);

			for (double t = T0 + stepT; t <= Ttarget; t += stepT) {

				StateVector = RungeKutta4order(StateVector, t, stepT, A_tb);
			}
			LatestStatesAll.StateVector[SV] = StateVector;
			LatestStatesAll.Atb[SV] = A_tb_dummy;
			LatestStatesAll.FrequencyNumber[SV] =
					glo_msg.nav.at(SV).FrequencyNumber;
			LatestStatesAll.SVClockBias[SV] = glo_msg.nav.at(SV).SVClockBias;
			LatestStatesAll.SVRelativeFrequencyBias[SV] =
					glo_msg.nav.at(SV).SVRelativeFrequencyBias;

			sats.push_back(SV);
		}
	}
	LatestStatesAll.epoch = TargetEpoch;
	LatestStatesAll.sats = sats;
	LatestStatesAll.SystemCorrectiontTme = glo_msg.header.SystemCorrectiontTme;
	return LatestStatesAll;
}

Orbits propagateOrbits(Orbits& InitOrbits, const Epoch& TargetEpoch,
		const int& stepT) {

	Matrix StateVectorUpd(6, 1, 0.0);

	double Tinit = InitOrbits.epoch.toSeconds();
	double Tmax = TargetEpoch.toSeconds();

	for (int& SV : InitOrbits.sats) {
		StateVectorUpd = InitOrbits.StateVector[SV];
		for (double t = Tinit; t <= Tmax; t += stepT) {
			StateVectorUpd = RungeKutta4order(StateVectorUpd, t, stepT,
					InitOrbits.Atb[SV]);
		}
		InitOrbits.StateVector[SV] = StateVectorUpd;
	}
	InitOrbits.epoch = TargetEpoch;
	return InitOrbits;
}

Matrix ECEFtoECI(const Matrix& vECEF, const Epoch& epoch_UTC) {

	const double We = 7.2921151467 * pow(10, -5); // mean Earth rotation Rate, w.r.t. vernal equinox, [rad/s]
	double Ang = We * epoch_UTC.toSeconds();

	int nRows = vECEF.getRows();
	Matrix vECI(nRows, 1, 0);

	vECI(0) = vECEF(0) * cos(Ang) - vECEF(1) * sin(Ang);
	vECI(1) = vECEF(0) * sin(Ang) + vECEF(1) * cos(Ang);
	vECI(2) = vECEF(2);

	if (nRows == 6) {
		vECI(3) = vECEF(3) * cos(Ang) - vECEF(4) * sin(Ang) - We * vECEF(1);
		vECI(4) = vECEF(3) * sin(Ang) + vECEF(4) * cos(Ang) + We * vECEF(0);
		vECI(5) = vECEF(5);
	}

	return vECI;
}

Matrix ECItoECEF(const Matrix& vECI, const Epoch& epoch_UTC) {

	const double We = 7.2921151467 * pow(10, -5); // mean Earth rotation Rate, w.r.t. vernal equinox, [rad/s]
	double Ang = We * epoch_UTC.toSeconds();

	int nRows = vECI.getRows();
	Matrix vECEF(nRows, 1, 0);

	vECEF(0) = vECI(0) * cos(Ang) + vECI(1) * sin(Ang);
	vECEF(1) = -vECI(0) * sin(Ang) + vECI(1) * cos(Ang);
	vECEF(2) = vECI(2);

	if (nRows == 6) {
		double vx = vECI(3) + We * vECI(1);
		double vy = vECI(4) - We * vECI(0);
		double vz = vECI(5);

		vECEF(3) = vx * cos(Ang) + vy * sin(Ang);
		vECEF(4) = -vx * sin(Ang) + vy * cos(Ang);
		vECEF(5) = vz;
	}

	return vECEF;
}

Matrix ECEFtoENU(const Matrix& Xecef, const Matrix& Xorigin) {

	Matrix Xenu(3, 1, 0);
	double latXoriginRad = atan2(Xorigin(1), Xorigin(0));
	double lonXoriginRad = atan2(Xorigin(2),
			sqrt(Xorigin(0) * Xorigin(0) + Xorigin(1) + Xorigin(1)));

	Matrix Q1(3, 3, 0);
	Q1(0, 0) = -1;
	Q1(1, 1) = 1;
	Q1(2, 2) = 1;

	Matrix d = Xecef - Xorigin;
	const double pi = boost::math::constants::pi<double>();

//	Matrix R_3(3,3,0);
//	R_3 = R3(latXoriginRad);
	Matrix dhori(3, 1, 0);
//	dhori = Q1 * ( R2(pi/2 - lonXoriginRad) * ( R3(latXoriginRad) * d));

	return dhori;
}

Matrix ENUtoAzimuthElevation(const Matrix& Xenu) {

	const double pi = boost::math::constants::pi<double>();
	Matrix AzElev(2, 1, 0);

	AzElev(0) = atan2(Xenu(1), Xenu(0)) * 180 / pi;
	AzElev(1) = 90 - (acos(Xenu(2) / norm(Xenu)) * 180 / pi);
	return AzElev;
}

Matrix ECEFtoAzimuthElevation(const Matrix& Xecef, const Matrix& Xorigin) {

	Matrix Xenu(3, 1, 0);
	Xenu = ECEFtoENU(Xecef, Xorigin);
	Matrix AzElev(2, 1, 0);
	AzElev = ENUtoAzimuthElevation(Xenu);
	return AzElev;
}

Matrix MSGtoTopo(const glonass_nav_msg& glo_msg, const Matrix& Xorigin) {

	Matrix AzElevStack(24, 2, 0);
	Matrix AzElev(2, 1, 0);
	Matrix Xecef(3, 1, 0);
	for (unsigned int i = 0; i < glo_msg.sats.size(); ++i) {
		Xecef = getStaveVectorX(glo_msg.nav.at(glo_msg.sats[i]));
		AzElev = ECEFtoAzimuthElevation(Xecef, Xorigin);
		AzElevStack(i, 0) = AzElev(0);
		AzElevStack(i, 1) = AzElev(1);
	}
	return AzElevStack;
}

void printStateVector(const Matrix& StateVector) {
	double r2 = StateVector(0) * StateVector(0)
			+ StateVector(1) * StateVector(1) + StateVector(2) * StateVector(2);
	double r = pow(r2, 0.5);

	double v2 = StateVector(3) * StateVector(3)
			+ StateVector(4) * StateVector(4) + StateVector(5) * StateVector(5);
	double v = pow(v2, 0.5);

	int w = 14;
	cout << "xyz, Vxyz : " << setw(w) << setprecision(3) << StateVector(0)
			<< " " << setw(w) << StateVector(1) << " " << setw(w)
			<< StateVector(2) << " ";
	w = 12;
	cout << setw(w) << StateVector(3) << " " << setw(w) << StateVector(4) << " "
			<< setw(w) << StateVector(5) << " r: " << setw(14) << r << " v: "
			<< setw(w) << v << endl;

}

