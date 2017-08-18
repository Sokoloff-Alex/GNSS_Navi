/*
 * RINEX.h
 *
 *  Created on: 12 Aug 2017
 *      Author: Sokol
 */

#ifndef RINEX_H_
#define RINEX_H_

#include "navmsg.h"
#include "Observations.h"

// parse observation file
// todo: output stream&
Observations parseRINEX(ifstream& textfile);

// get observation for one epoch only
Observations parseRINEX_Epoch(ifstream& textfile);

Epoch parseObsEpoch(const string& str);

#endif /* RINEX_H_ */
