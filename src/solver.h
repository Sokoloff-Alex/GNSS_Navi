/*
 * solver.h
 *
 *  Created on: 16 Aug 2017
 *      Author: Sokol
 */

#ifndef SOLVER_H_
#define SOLVER_H_

#include "Observations.h"
#include "OrbProp.h"

Matrix<double> solverLS(const Observations& obs, const Orbits& orbs, const Matrix<double>& RecXYZapriori);

void printXYZT(const Matrix<double>& rec_est);

#endif /* SOLVER_H_ */
