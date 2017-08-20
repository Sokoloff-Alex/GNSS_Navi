/*
 * RINEXnav.h
 *
 *  Created on: 13 Aug 2017
 *      Author: Sokol
 */

#ifndef RINEXNAV_H_
#define RINEXNAV_H_

#include "navmsg.h"

class RINEXnav {
};

// parse navigation file
// todo: output stream&
glonass_nav_msg parseRINEX_Nav(ifstream& nav_textfile);

// todo parse navigation file
// start with for GLONASS (easiest)
glonass_nav_msg parseRINEX_NavEpoch(ifstream& nav_textfile);

glonass_nav_msg& getGLONASSnavmsgBlock(ifstream& textfile, glonass_nav_msg& glo_nav_msg);

void printGLOnavmsg(const glonass_nav_msg& glo_msg);

double stoDouble(const string& strValue, const string& strFormat);

double stoDouble(const string& strValue);


#endif /* RINEXNAV_H_ */
