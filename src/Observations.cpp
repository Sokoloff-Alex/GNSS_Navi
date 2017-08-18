/*
 * Observations.cpp
 *
 *  Created on: 15 Aug 2017
 *      Author: Sokol
 */

#include <iostream>
#include <iomanip>
#include <set>
#include "Observations.h"
#include "navmsg.h"

void printRanges(const Observations& obs) {
	for (auto& System : obs.ranges) {
		for (auto& RangeEntry : System.second) {
			cout << System.first << fixed << setw(2) << RangeEntry.first << " "
					<< setprecision(3) << RangeEntry.second << endl;
		}
	}
}

void printRanges(const Observations& obs, const string& Sys) {
	for (auto System : obs.ranges) {
		if (System.first == Sys) {
			for (auto RangeEntry : System.second) {
				cout << System.first << fixed << setw(2) << RangeEntry.first << " "
						<< setprecision(3) << RangeEntry.second << endl;
			}
		}
	}
}

void printSet(const set<int>& s) {
	for (auto& elem : s) {
		cout << elem << " ";
	}
	cout << endl;
}

set<int> intersect(const Observations& obs, const Orbits& orbs, const string& Sys) {

	set<int> setObs(obs.sats.at(Sys).begin(), obs.sats.at(Sys).end());
	set<int> setOrbs(orbs.sats.begin(), orbs.sats.end());
	set<int> setCommon;

	for (auto& sat : setObs) {
		if (setOrbs.count(sat)) {
			setCommon.insert(sat);
		}
	}
	return setCommon;
}

Observations select(const Observations& obs, const set<int>& sats, const string& sys) {
	Observations obs_selected;
	map<string, map<int, double> > Rranges;
	for (int sat : sats) {
		Rranges[sys][sat] = obs.ranges.at(sys).at(sat);
		obs_selected.sats[sys].push_back(sat);
	}
	obs_selected.ranges = Rranges;
	obs_selected.epoch = obs.epoch;
	return obs_selected;
}

Matrix<double> getRanges(const Observations& obs) {
	int size = obs.ranges.at("R").size();
	Matrix<double> ranges(size, 1, 0);
	int i = 0;
	for (auto& item : obs.ranges.at("R")) {
		ranges(i, 0) = item.second;
		i++;
	}
	return ranges;
}

