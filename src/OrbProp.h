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


class RK4 {
};

Matrix<double> RungeKutta4order(const vector<double>& initialState, const double& timeStep, const Matrix<double>& AccelAtTb);

Matrix<double> getDerivatives(const double& t, const Matrix<double>& StateVector, const Matrix<double>& AccelerationAtTb);

Orbits propagateOrbits(const glonass_nav_msg& glo_msg, const Epoch& TargetEpoch);

Orbits propagateOrbits(Orbits& InitOrbits, const Epoch& TargetEpoch, const int& stepT);

Matrix<double> ECEFtoECI(const Matrix<double>& vECEF, const Epoch& epoch_UTC);
Matrix<double> ECItoECEF(const Matrix<double>& vECI, const Epoch& epoch_UTC);

void printStateVector(const Matrix<double>& StateVector);


#endif /* ORBPROP_H_ */
