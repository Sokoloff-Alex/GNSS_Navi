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

Matrix solverLS(const Observations& obsAll, const Orbits& orbsAll, const Matrix& RecXYZapriori);

void printXYZT(const Matrix& rec_est);

#endif /* SOLVER_H_ */
