/*
 * Observations.h
 *
 *  Created on: 15 Aug 2017
 *      Author: Sokol
 */

#ifndef OBSERVATIONS_H_
#define OBSERVATIONS_H_

#include <set>
#include "navmsg.h"

struct Observations {
public:
	Epoch epoch;
//     system       sv   range
	map<string, map<int, double> > ranges;
	map<string, vector<int> > sats;
};

void printRanges(const Observations& obs);

void printRanges(const Observations& obs, const string& Sys);

set<int> intersect(const Observations& obs, const Orbits& orbs,
		const string& Sys);

Observations select(const Observations& obs, const set<int>& sats,
		const string& sys);

Matrix getRanges(const Observations& obs);

#endif /* OBSERVATIONS_H_ */
