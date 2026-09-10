/**
 * @file realphases.h
 * @brief The real-skeleton phases: every user-reported FBIK behaviour, reproduced on a dumped
 *        figure skeleton through the simulator and gated on the measurements that once failed.
 */

#ifndef IKHARNESS_REALPHASES_H
#define IKHARNESS_REALPHASES_H

#include "armature.h"
#include "report.h"

#include <vector>

namespace ikharness {

/// Runs every real-figure phase the report wants (see Report::wants) on @p bones.
void realPhases(Report& report, const std::vector<pose::ArmatureBone>& bones);

} // namespace ikharness

#endif // IKHARNESS_REALPHASES_H
