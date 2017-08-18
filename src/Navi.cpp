//============================================================================
// Name        : Navi.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <set>

#include "Matrix.h"
#include "RINEX.h"
#include "RINEXnav.h"
#include "OrbProp.h"
#include "navmsg.h"
#include "Observations.h"
#include "solver.h"

using namespace std;

int main() {

	const double c = 299792458; // [m/s]
	int w = 16;

	string obs_file_path =
			"D:/Dev/GNSS/obs/WTZZ00DEU_S_20171440000_15M_01S_MO.00o";
	string RN_file_path =
			"D:/Dev/GNSS/obs/WTZ200DEU_R_20171440000_01D_RN.rnx/WTZ200DEU_R_20171440000_01D_RN.RNX";
	string out_file_path = "D:/Dev/GNSS/obs/xyzt.txt";

	try {

		ofstream out_file(out_file_path);

		ifstream obs_file(obs_file_path);
		Observations obs = parseRINEX(obs_file);

		ifstream RN_file(RN_file_path);
		glonass_nav_msg glo_msg = parseRINEX_Nav(RN_file);

		Orbits orbs = propagateOrbits(glo_msg, obs.epoch);
		set<int> commonSV = intersect(obs, orbs, "R");
		Observations obsR = select(obs, commonSV, "R");
		Orbits orbR = select(orbs, commonSV);
		Matrix<double> RecXYZ(3, 1, 0.0);
		Matrix<double> rec_est = solverLS(obsR, orbR, RecXYZ);
		RecXYZ(0) = rec_est(0);
		RecXYZ(1) = rec_est(1);
		RecXYZ(2) = rec_est(2);
		out_file << fixed << setprecision(3) << setw(w) << rec_est(0) << setw(w)
				<< rec_est(1) << setw(w) << rec_est(2) << setw(w)
				<< setprecision(9) << rec_est(3) / c << setw(w)
				<< setprecision(1) << obs.epoch.toSeconds() << setw(5)
				<< setprecision(0) << commonSV.size() << endl;

		for (int sec = 0; sec < 60 * 15 - 1; ++sec) {
			Observations obs = parseRINEX_Epoch(obs_file);
			orbs = propagateOrbits(orbs, obs.epoch, 1);
			commonSV = intersect(obs, orbs, "R");
			obsR = select(obs, commonSV, "R");
			orbR = select(orbs, commonSV);
			rec_est = solverLS(obsR, orbR, RecXYZ);

			out_file << fixed << setprecision(3) << setw(w) << rec_est(0) << setw(w)
					<< rec_est(1) << setw(w) << rec_est(2) << setw(w)
					<< setprecision(9) << rec_est(3) / c << setw(w)
					<< setprecision(1) << obs.epoch.toSeconds() << setw(5)
					<< setprecision(0) << commonSV.size() << endl;
		}

		obs_file.close();
		RN_file.close();
		out_file.flush();
		out_file.close();

	} catch (exception& e) {
		cerr << "Error!" << endl;
		e.what();
	}

	return 0;
}
