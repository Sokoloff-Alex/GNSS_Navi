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

	string obs_file_path =
			"D:/Dev/GNSS/obs/2017/144/WTZR00DEU_R_20171440000_01D_30S_MO.00o";
	string RN_file_path =
			"D:/Dev/GNSS/obs/2017/144/WTZ200DEU_R_20171440000_01D_RN.rnx/WTZ200DEU_R_20171440000_01D_RN.RNX";
	string out_file_path = "D:/Dev/GNSS/obs/xyzt.txt";

	Matrix<double> RecXYZ(3, 1, 0.0);
	RecXYZ(0) = 4075580.848;
	RecXYZ(1) = 931853.570;
	RecXYZ(2) = 4801567.925;

	try {

		ifstream obs_file(obs_file_path);
		Observations obs = parseRINEX(obs_file);

		ifstream RN_file(RN_file_path);
		glonass_nav_msg glo_msg = parseRINEX_Nav(RN_file);

		Orbits orbs = propagateOrbits(glo_msg, obs.epoch);
		Matrix<double> rec_est = solverLS(obs, orbs, RecXYZ);
		RecXYZ(0) = rec_est(0);
		RecXYZ(1) = rec_est(1);
		RecXYZ(2) = rec_est(2);



		for (int sec = 0; sec < 90; sec+=30) {

			obs = parseRINEX_Epoch(obs_file);
			Epoch currEpoch = obs.epoch;
			if (currEpoch.seconds == 0 && (currEpoch.minutes == 15 || currEpoch.minutes == 45)) {
//				glo_msg = parseRINEX_Nav(RN_file);
			}
			orbs = propagateOrbits(orbs, obs.epoch, 1);
			rec_est = solverLS(obs, orbs, RecXYZ);
			RecXYZ(0) = rec_est(0);
			RecXYZ(1) = rec_est(1);
			RecXYZ(2) = rec_est(2);
		}

		obs_file.close();
		RN_file.close();

	} catch (exception& e) {
		cerr << "Error!" << endl;
		e.what();
	}

	return 0;
}
