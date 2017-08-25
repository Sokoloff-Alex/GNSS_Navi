/*
 * OrbProp.h
 *
 *  Created on: 14 Aug 2017
 *      Author: Sokol
 */

#ifndef ORBPROP_H_
#define ORBPROP_H_

#include "Matrix.h"
#include "RINEXnav.h"

using namespace std;

Matrix RungeKutta4order(const vector<double>& initialState, const double& timeStep, const Matrix& AccelAtTb);

Matrix getDerivatives(const double& t, const Matrix& StateVector, const Matrix& AccelerationAtTb);

Orbits propagateOrbits(const glonass_nav_msg& glo_msg, const Epoch& TargetEpoch);

Orbits propagateOrbits(Orbits& InitOrbits, const Epoch& TargetEpoch, const int& stepT);

Matrix ECEFtoECI(const Matrix& vECEF, const Epoch& epoch_UTC);

Matrix ECItoECEF(const Matrix& vECI, const Epoch& epoch_UTC);

Matrix ENUtoAzimuthElevation(const Matrix& enu);

Matrix ECEFtoAzimuthElevation(const Matrix& Xecef, const Matrix& Xorigin);

Matrix MSGtoTopo(const glonass_nav_msg& glo_msg, const Matrix& Xorigin);

void printStateVector(const Matrix& StateVector);


#endif /* ORBPROP_H_ */
