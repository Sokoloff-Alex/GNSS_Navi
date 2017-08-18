/* OrbProp.cpp
 *
 * Created on: 14 Aug 2017
 * Author: Sokol
 */

#include "OrbProp.h"

#include <vector>
#include <iostream>
#include <iomanip>
#include <math.h>
#include "Matrix.h"
#include "RINEXnav.h"

using namespace std;

void printStateVector(const Matrix<double>& StateVector) {
	double r2 = StateVector(0, 0) * StateVector(0, 0) + StateVector(1, 0) * StateVector(1, 0)
			+ StateVector(2, 0) * StateVector(2, 0);
	double r = pow(r2, 0.5);

	double v2 = StateVector(3, 0) * StateVector(3, 0) + StateVector(4, 0) * StateVector(4, 0)
			+ StateVector(5, 0) * StateVector(5, 0);
	double v = pow(v2, 0.5);

	int w = 14;
	cout << "xyz, Vxyz : " << setw(w) << setprecision(3) << StateVector(0, 0) << " " << setw(w)
			<< StateVector(1, 0) << " " << setw(w) << StateVector(2, 0) << " ";
	w = 12;
	cout << setw(w) << StateVector(3, 0) << " " << setw(w) << StateVector(4, 0) << " " << setw(w)
			<< StateVector(5, 0) << " r: " << setw(14) << r << " v: " << setw(w) << v << endl;

}

//find first time derivatives of state vector (oblate Earth model)
Matrix<double> getDerivatives(const double& t, const Matrix<double>& StateVector,
		const Matrix<double>& A_Tb) {

	const double GM = 398600441.8 * pow(10, 6); // geocentric constant of Earth Gravity field (including Atmosphere), [m^3 / s^2]
	const double Ae = 6378136; // large (equatorial) semi-axis of ellipsoid, [m]
	const double J02 = 1082625.75 * pow(10, -9); // zonal harmonic coeff. of 2nd order (oblatness) [dimentionless]
	const double OmegaE = 7.2921151467 * pow(10, -5); // mean Earth rotation Rate, w.r.t. vernal equinox, [rad/s]

	double x = StateVector(0, 0);
	double y = StateVector(1, 0);
	double z = StateVector(2, 0);

	double r2 = (x * x + y * y + z * z);
	double r = pow(r2, 0.5);

	Matrix<double> A(3, 1, 0.0);

	double z2_r2 = z * z / r2;

	A(0, 0) = 5 * z2_r2 - 1;
	A(1, 0) = 5 * z2_r2 - 1;
	A(2, 0) = 5 * z2_r2 - 3;

	Matrix<double> Flattening(3, 1, 1.0);
	Flattening -= A * (3 / 2 * J02 * Ae * Ae / r2);

	Matrix<double> StateVectorDot(6, 1, 0.0);

	StateVectorDot(0, 0) = StateVector(3, 0);
	StateVectorDot(1, 0) = StateVector(4, 0);
	StateVectorDot(2, 0) = StateVector(5, 0);

	double GM_by_r3 = GM / (r2 * r);
	StateVectorDot(3, 0) = -GM_by_r3 * StateVector(0, 0) * Flattening(0, 0);
	StateVectorDot(4, 0) = -GM_by_r3 * StateVector(1, 0) * Flattening(1, 0);
	StateVectorDot(5, 0) = -GM_by_r3 * StateVector(2, 0) * Flattening(2, 0);

	// account Earth Rotation for PZ-90 system
	StateVectorDot(3, 0) += OmegaE * OmegaE * x + 2 * OmegaE * StateVectorDot(1, 0) + A_Tb(0, 0);
	StateVectorDot(4, 0) += OmegaE * OmegaE * y - 2 * OmegaE * StateVectorDot(0, 0) + A_Tb(1, 0);

	return StateVectorDot;
}

Matrix<double> RungeKutta4order(const Matrix<double>& Y, const double& t, const double& h,
		const Matrix<double>& Atb) {

	Matrix<double> k1 = getDerivatives(t, Y, Atb) * h;
	Matrix<double> k2 = getDerivatives(t + h / 2, k1 * h / 2 + Y, Atb);
	Matrix<double> k3 = getDerivatives(t + h / 2, k2 * h / 2 + Y, Atb);
	Matrix<double> k4 = getDerivatives(t + h, k3 * h + Y, Atb);

	Matrix<double> Ynew = (k1 + k2 * 2 + k3 * 2 + k4) / 6 + Y;
	return Ynew;
}

Orbits propagateOrbits(const glonass_nav_msg& glo_msg, const Epoch& TargetEpoch) {

	vector<int> sats;
	Orbits LatestStatesAll;
	Matrix<double> StateVector(6, 1, 0.0);
	Matrix<double> A_at_Tb(3, 1, 0.0);
	for (auto& msgSV : glo_msg.nav) {
		Epoch EphemerisEpoch = msgSV.second.epoch;
		if (TargetEpoch.hour == 0 && TargetEpoch.minutes <= 15) {
			if (EphemerisEpoch.hour == 23 && EphemerisEpoch.minutes >= 30) { // propagate only recent satellites
				int SV = msgSV.first;
//				cout << "Propagating SV " << SV << " propagate from last day" << endl;
				StateVector = getStaveVectorXV(msgSV.second);
				A_at_Tb = getStaveVectorA(msgSV.second);

				double T0 = -15 * 60; // last 15 minutes only, keep negative, [sec]
				double stepT = 1; // [sec]
				for (double t = T0 + stepT; t <= 0; t += stepT) {
					StateVector = RungeKutta4order(StateVector, t, stepT, A_at_Tb);
				}
				LatestStatesAll.StateVector[SV] = StateVector;
				sats.push_back(SV);
			}
		} else {
			int SV = msgSV.first;
			cout << "Propagating SV " << SV << endl;
			StateVector = getStaveVectorXV(msgSV.second);
			A_at_Tb = getStaveVectorA(msgSV.second);

			double T0 = 0;
			double stepT = 1; // [sec]
			for (double t = T0 + stepT; t <= TargetEpoch.toSeconds(); t += stepT) {
				StateVector = RungeKutta4order(StateVector, t, stepT, A_at_Tb);
			}
			LatestStatesAll.StateVector[SV] = StateVector;
			LatestStatesAll.Atb[SV] = A_at_Tb;

			sats.push_back(SV);
		}
	}
	LatestStatesAll.epoch = TargetEpoch;
	LatestStatesAll.sats = sats;
	return LatestStatesAll;
}

Orbits propagateOrbits(Orbits& InitOrbits, const Epoch& TargetEpoch, const int& stepT) {

	Matrix<double> StateVectorUpd(6, 1, 0.0);

	double Tinit = InitOrbits.epoch.toSeconds();
	double Tmax = TargetEpoch.toSeconds();

	for (int& SV : InitOrbits.sats) {
		StateVectorUpd = InitOrbits.StateVector[SV];
		for (double t = Tinit; t <= Tmax; t += stepT) {
			StateVectorUpd = RungeKutta4order(StateVectorUpd, t, stepT, InitOrbits.Atb[SV]);
		}
		InitOrbits.StateVector[SV] = StateVectorUpd;
	}
	InitOrbits.epoch = TargetEpoch;
	return InitOrbits;
}
