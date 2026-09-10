/**
 * @file unitphases.h
 * @brief The harness's unit phases: the IK math and constraint primitives, tested without a
 *        skeleton (Euler round-trips for all six rotation orders incl. gimbal lock, hinge and
 *        asymmetric-cone clamps, graph re-rooting, the support polygon).
 */

#ifndef IKHARNESS_UNITPHASES_H
#define IKHARNESS_UNITPHASES_H

#include "report.h"

namespace ikharness {

void testEulerRoundTrip(Report& report);
void testConstraints(Report& report);
void testSkeletonGraph(Report& report);
void testBalance(Report& report);

} // namespace ikharness

#endif // IKHARNESS_UNITPHASES_H
