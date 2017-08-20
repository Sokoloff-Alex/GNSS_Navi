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

using namespace std;

//find first time derivatives of state vector (oblate Earth model)
Matrix<double> getDerivatives(const double& t,
		const Matrix<double>& StateVector, const Matrix<double>& Ams) {

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

	Matrix<double> StateVecDot(6, 1, 0.0);

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

Matrix<double> RungeKutta4order(const Matrix<double>& Y, const double& t,
		const double& h, const Matrix<double>& Atb) {

	Matrix<double> k1 = getDerivatives(t, Y, Atb) * h;
	Matrix<double> k2 = getDerivatives(t + h / 2, k1 * h / 2 + Y, Atb);
	Matrix<double> k3 = getDerivatives(t + h / 2, k2 * h / 2 + Y, Atb);
	Matrix<double> k4 = getDerivatives(t + h, k3 * h + Y, Atb);

	Matrix<double> Ynew = (k1 + k2 * 2 + k3 * 2 + k4) / 6 + Y;
	return Ynew;
}

Orbits propagateOrbits(const glonass_nav_msg& glo_msg,
		const Epoch& TargetEpoch) {

	ofstream file_orb("D:/Dev/GNSS/obs/orbits.txt");

	vector<int> sats;
	Orbits LatestStatesAll;
	Matrix<double> StateVector(6, 1, 0.0);
	Matrix<double> A_at_Tb(3, 1, 0.0);
//	Matrix<double> StateVectorECI(6, 1, 0.0);
//	Matrix<double> A_at_Tb_ECI(3, 1, 0.0);
	Matrix<double> A_at_Tb_dummy(6, 1, 0.0);
	for (auto& msgSV : glo_msg.nav) {
		Epoch EphemerisEpoch = msgSV.second.epoch;
		if (TargetEpoch.hour == 0 && TargetEpoch.minutes <= 15) {
			if (EphemerisEpoch.hour == 23 && EphemerisEpoch.minutes >= 30) { // propagate only recent satellites
				int SV = msgSV.first;
//				cout << "Propagating SV " << SV << " propagate from last day" << endl;
				StateVector = getStaveVectorXV(msgSV.second);
				A_at_Tb = getStaveVectorA(msgSV.second);
				A_at_Tb_dummy(0) = A_at_Tb(0);
				A_at_Tb_dummy(1) = A_at_Tb(1);
				A_at_Tb_dummy(2) = A_at_Tb(2);

				double T0 = -15 * 60; // last 15 minutes only, keep negative, [sec]
				double stepT = 1; // [sec]
				for (double t = T0 + stepT; t <= 0; t += stepT) {

					{ // check propagation
						int w = 15;
						double x_norm = sqrt(
								StateVector(0) * StateVector(0)
										+ StateVector(1) * StateVector(1)
										+ StateVector(2) * StateVector(2));

						double v_norm = sqrt(
								StateVector(3) * StateVector(3)
										+ StateVector(4) * StateVector(4)
										+ StateVector(5) * StateVector(5));

						file_orb << setw(2) << SV << " " << fixed << setw(12) << setprecision(1)
								<< t
								<< setprecision(3) << setw(w) << StateVector(0)
								<< setw(w) << StateVector(1) << setw(w)
								<< StateVector(2) << setw(w) << StateVector(3)
								<< setw(w) << StateVector(4) << setw(w)
								<< StateVector(5) << setw(w) << x_norm
								<< setw(w) << v_norm << endl;
					}
//
//					StateVectorECI = ECEFtoECI(StateVector, EphemerisEpoch);
//					A_at_Tb_ECI = ECEFtoECI(A_at_Tb, EphemerisEpoch);
//					StateVectorECI = RungeKutta4order(StateVectorECI, t, stepT, A_at_Tb_ECI);
//					StateVector = ECItoECEF(StateVectorECI, EphemerisEpoch);
					StateVector = RungeKutta4order(StateVector, t, stepT, A_at_Tb);
				}
				LatestStatesAll.StateVector[SV] = StateVector;
				LatestStatesAll.Atb[SV] = A_at_Tb_dummy;
				LatestStatesAll.FrequencyNumber[SV] = glo_msg.nav.at(SV).FrequencyNumber;
				LatestStatesAll.SVClockBias[SV] = glo_msg.nav.at(SV).SVClockBias;
				LatestStatesAll.SVRelativeFrequencyBias[SV] = glo_msg.nav.at(SV).SVRelativeFrequencyBias;

				sats.push_back(SV);
			}
		} else {
			int SV = msgSV.first;
			cout << "Propagating SV " << SV << endl;
			StateVector = getStaveVectorXV(msgSV.second);
			A_at_Tb = getStaveVectorA(msgSV.second);
			A_at_Tb_dummy(0) = A_at_Tb(0);
			A_at_Tb_dummy(1) = A_at_Tb(1);
			A_at_Tb_dummy(2) = A_at_Tb(2);

			double T0 = 0;
			double stepT = 1; // [sec]
			for (double t = T0 + stepT; t <= TargetEpoch.toSeconds(); t += stepT) {
//				StateVectorECI = ECEFtoECI(StateVector, EphemerisEpoch);
//				A_at_Tb_ECI = ECEFtoECI(A_at_Tb, EphemerisEpoch);
//				StateVectorECI = RungeKutta4order(StateVectorECI, t, stepT, A_at_Tb_ECI);
//				StateVector = ECItoECEF(StateVectorECI, EphemerisEpoch);
				StateVector = RungeKutta4order(StateVector, t, stepT, A_at_Tb);

				{ // check propagation
					int w = 15;
					double x_norm = sqrt(
							StateVector(0) * StateVector(0)
									+ StateVector(1) * StateVector(1)
									+ StateVector(2) * StateVector(2));

					double v_norm = sqrt(
							StateVector(3) * StateVector(3)
									+ StateVector(4) * StateVector(4)
									+ StateVector(5) * StateVector(5));

					file_orb << setw(2) << SV << " " << fixed << setw(12) << setprecision(1)
							<< t
							<< setprecision(3) << setw(w) << StateVector(0)
							<< setw(w) << StateVector(1) << setw(w)
							<< StateVector(2) << setw(w) << StateVector(3)
							<< setw(w) << StateVector(4) << setw(w)
							<< StateVector(5) << setw(w) << x_norm
							<< setw(w) << v_norm << endl;
				}

			}
			LatestStatesAll.StateVector[SV] = StateVector;
			LatestStatesAll.Atb[SV] = A_at_Tb_dummy;
			LatestStatesAll.FrequencyNumber[SV] = glo_msg.nav.at(SV).FrequencyNumber;
			LatestStatesAll.SVClockBias[SV] = glo_msg.nav.at(SV).SVClockBias;
			LatestStatesAll.SVRelativeFrequencyBias[SV] = glo_msg.nav.at(SV).SVRelativeFrequencyBias;

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

Matrix<double> ECEFtoECI(const Matrix<double>& vECEF, const Epoch& epoch_UTC) {

	const double We = 7.2921151467 * pow(10, -5); // mean Earth rotation Rate, w.r.t. vernal equinox, [rad/s]
	double Ang = We * epoch_UTC.toSeconds();

	int nRows = vECEF.getRows();
	Matrix<double> vECI(nRows, 1, 0);

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

Matrix<double> ECItoECEF(const Matrix<double>& vECI, const Epoch& epoch_UTC) {

	const double We = 7.2921151467 * pow(10, -5); // mean Earth rotation Rate, w.r.t. vernal equinox, [rad/s]
	double Ang = We * epoch_UTC.toSeconds();

	int nRows = vECI.getRows();
	Matrix<double> vECEF(nRows, 1, 0);

	vECEF(0) =  vECI(0) * cos(Ang) + vECI(1) * sin(Ang);
	vECEF(1) = -vECI(0) * sin(Ang) + vECI(1) * cos(Ang);
	vECEF(2) =  vECI(2);

	if (nRows == 6) {
		double vx = vECI(3) + We * vECI(1);
		double vy = vECI(4) - We * vECI(0);
		double vz = vECI(5);

		vECEF(3) =  vx * cos(Ang) + vy * sin(Ang);
		vECEF(4) = -vx * sin(Ang) + vy * cos(Ang);
		vECEF(5) =  vz;
	}

	return vECEF;
}

void printStateVector(const Matrix<double>& StateVector) {
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

