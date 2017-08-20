/*
 * solver.cpp
 *
 *  Created on: 16 Aug 2017
 *      Author: Sokol
 */

#include <math.h>
#include <iostream>
#include <iomanip>
#include "solver.h"
#include "Observations.h"
#include "OrbProp.h"
#include "Matrix.h"

using namespace std;

Matrix<double> solverLS(const Observations& obs, const Orbits& orbs,
		const Matrix<double>& RecXYZapriori) {
	vector<int> sats = orbs.sats;
	int numberOfSats = sats.size();

	if (numberOfSats < 4) {
		return Matrix<double>(4, 1, 0.0);
	}
	// constants, all units in SI !!!
	const double c = 299792458; // [m/s]
	const double Omega_E_rate = 0.000072921151467; //[rad/s]
	const double Tzpd = 2.3; // [m]

	Matrix<double> RecXYZ(3, 1, 0);
	RecXYZ = RecXYZapriori;

	Matrix<double> Range = getRanges(obs);
	Matrix<double> Xs(3, 1, 0);
	Matrix<double> Vs(3, 1, 0);

	Orbits orbs_cor = orbs;

	for (int iSat = 0; iSat < numberOfSats; ++iSat) {

		Xs(0) = orbs.StateVector.at(sats[iSat])(0);
		Xs(1) = orbs.StateVector.at(sats[iSat])(1);
		Xs(2) = orbs.StateVector.at(sats[iSat])(2);
		Vs(0) = orbs.StateVector.at(sats[iSat])(3);
		Vs(1) = orbs.StateVector.at(sats[iSat])(4);
		Vs(2) = orbs.StateVector.at(sats[iSat])(5);

		// Correction for Satellite motion during signal travel time
		double dt_rs = Range(iSat) / c;
		Xs -= Vs * dt_rs;

		// Correction for Earth rotation during signal travel time
		double dOmega = Omega_E_rate * dt_rs;

		Xs(0) = cos(dOmega) * Xs(0) + sin(dOmega) * Xs(1);
		Xs(1) = -sin(dOmega) * Xs(0) + cos(dOmega) * Xs(1);

		// propagate Ephemeris from Tsv to Tuts
		double TauN = orbs_cor.SVClockBias.at(sats[iSat]);
		double GammaN = orbs_cor.SVRelativeFrequencyBias.at(sats[iSat]);
		double TauC = orbs_cor.SystemCorrectiontTme;
		double tSV = orbs_cor.epoch.toSeconds();

		double SVtoUT = (TauN - GammaN * tSV + TauC);
		Matrix<double> dXs_dts(3, 1, 0);
		dXs_dts = Vs * SVtoUT;
		Xs -= dXs_dts;
//		cout << "dXs_dts = " << endl;
//		dXs_dts.print();

		orbs_cor.StateVector.at(sats[iSat])(0) = Xs(0);
		orbs_cor.StateVector.at(sats[iSat])(1) = Xs(1);
		orbs_cor.StateVector.at(sats[iSat])(2) = Xs(2);
	}

	// dummies
	Matrix<double> PRangeTilde(numberOfSats, 1, 0.0);
	Matrix<double> A(numberOfSats, 4, 1);  //designMatrix
	Matrix<double> Residuals(numberOfSats, 1, 0);
	Matrix<double> rec_est(4, 1, 0);

	int maxIter = 1;
	if (norm(RecXYZ) == 0) {
		maxIter = 5;
	}
	for (int iteration = 0; iteration < 5; ++iteration) {
		for (int iSat = 0; iSat < numberOfSats; ++iSat) {

			Xs(0) = orbs_cor.StateVector.at(sats[iSat])(0);
			Xs(1) = orbs_cor.StateVector.at(sats[iSat])(1);
			Xs(2) = orbs_cor.StateVector.at(sats[iSat])(2);
			Vs(0) = orbs_cor.StateVector.at(sats[iSat])(3);
			Vs(1) = orbs_cor.StateVector.at(sats[iSat])(4);
			Vs(2) = orbs_cor.StateVector.at(sats[iSat])(5);

			// unit vectors
			Matrix<double> e_sr = (Xs - RecXYZ) / norm(Xs - RecXYZ);
			A(iSat, 0) = e_sr(0);
			A(iSat, 1) = e_sr(1);
			A(iSat, 2) = e_sr(2);

			// Troposphere correction
			Matrix<double> e_rec(3, 1, 0);
			double cos_z = 0;
			double T = Tzpd;
			if (norm(RecXYZ) != 0) {
				e_rec = RecXYZ / norm(RecXYZ);
				cos_z = (e_rec.transpose() * e_sr)(0, 0);
				T = Tzpd / cos_z;
			}

			// Relativistic correction
			double dt_rel = -2 * (Xs.transpose() * Vs)(0, 0) / (c * c);

			// sat clock correction
			double TauN = orbs_cor.SVClockBias.at(sats[iSat]);
			double GammaN = orbs_cor.SVRelativeFrequencyBias.at(sats[iSat]);
			double TauC = orbs_cor.SystemCorrectiontTme;
			double tSV = orbs_cor.epoch.toSeconds();

//			double SVtoUT = (TauN - GammaN * tSV + TauC);
//			cout << "dts * c = " << fixed << setw(15) 	<< (TauN - GammaN * tSV + TauC) * c << endl;

			// move known values to lhs
			PRangeTilde(iSat) = Range(iSat) + (e_sr.transpose() * Xs)(0, 0)
					+ (TauN - GammaN * tSV + TauC) * c + dt_rel * c - T;
		}

		// LSE solution
		rec_est = (A.transpose() * A).inverse() * (A.transpose() * PRangeTilde);
		RecXYZ(0) = rec_est(0);
		RecXYZ(1) = rec_est(1);
		RecXYZ(2) = rec_est(2);
//		cout << " Range" << endl;
//		Range.print();
//		cout << " Range + knowns" << endl;
//		PRangeTilde.print();
//		cout << "Estimates" << endl;
		printXYZT(rec_est);
//		double r = sqrt(rec_est(0) * rec_est(0) + rec_est(1) * rec_est(1) + rec_est(2) * rec_est(2));
//		cout << "r = " << r << endl;
		Matrix<double> Resuduals(numberOfSats,1,0);
		Resuduals = PRangeTilde - A * rec_est;
		cout << "Residuals" << endl;
		Resuduals.print();
	}
	return rec_est;
}

void printXYZT(const Matrix<double>& rec_est) {
	const double c = 299792458; // [m/s]
	int w = 16;
	cout << fixed << setprecision(3) << setw(w) << rec_est(0) << " " << setw(w)
			<< rec_est(1) << " " << setw(w) << rec_est(2) << " " << setw(w)
			<< rec_est(3) / c << endl;
}

