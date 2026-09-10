/**
 * @file ikrigsolve.cpp
 * @brief The IkRig's per-tick solves: solveDrag (the drag goal against the pins, shaped by the
 *        drag's INTENT — a downward push yields the root into a crouch, an upward limb pull
 *        recruits the arm only, a sustained beyond-reach lift enters SUSPENSION — and blended
 *        with the continuous auto-balance re-solve) and the release settle (beginSettle /
 *        settleToPins).
 *
 * solveDrag runs once per 60 Hz drag tick with a deliberately small iteration budget: convergence
 * happens ACROSS ticks and the visible motion is the iteration process itself, damped by the
 * Armature's governors. The balance re-solve runs every tick on a copy and is blended in by a
 * low-passed ENGAGEMENT factor rather than toggled — every on/off variant measurably trembled.
 * The settle holds the release pose as its prior (never the drag-start pose) and pins the
 * released joint only for LIMB gestures, so the body can drop onto its feet after a trunk drag.
 * See ikrig.cpp for the TU layout; the constants come from ikrig_constants.h. Qt-free (std +
 * GLM).
 */

#include "ikrig.h"

#include "balancecontroller.h"
#include "ikrig_constants.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace pose {

void IkRig::beginSettle(const std::vector<glm::vec3>& positions) {
    const int n = m_graph.nodeCount();
    if (!dragActive() || static_cast<int>(positions.size()) != n) {
        return;
    }
    // A step still in flight at mouse-up completes instantly in TARGET terms: the pin lands at
    // its balanced spot and the settle's capped monotone rounds animate the foot down onto it —
    // leaving the swing target mid-air would settle the foot hanging there.
    if (m_stepPin >= 0) {
        landStep(positions);
    }
    // The settle's prior is the pose at MOUSE-UP — the pose the user just made and expects to
    // keep. (The drag-start pose would be wrong here: a prior toward it stood the figure back up
    // out of its crouch and yanked the placed hand toward where the drag began — the round-5
    // failure that originally led to running the settle prior-free. Prior-free was wrong too:
    // with only the pins guarded, the solver paid for millimeters of pin progress with
    // centimeters of unrelated drift, and a lifted foot visibly sagged after release.)
    m_settlePrior = positions;
    if (m_effector >= 0) {
        m_settleEffectorTarget = positions[static_cast<std::size_t>(m_effector)];
    }
    // Stiffness: everything not serving a pin holds the release pose firmly; pin-serving chains
    // (and the effector's own chain — it must articulate to keep the effector fixed while the
    // root shifts to plant the feet) stay nearly prior-free. Effectors are prior-exempt in the
    // solver, so the held joints themselves are unaffected.
    m_settlePriorWeights.assign(static_cast<std::size_t>(n), kPosePriorWeight * 2.0f);
    for (const IkEffector& pin : m_pins) {
        for (int cur = pin.node; cur >= 0 && cur != m_pelvis;
             cur = m_parents[static_cast<std::size_t>(cur)]) {
            m_settlePriorWeights[static_cast<std::size_t>(cur)] = kPosePriorWeight * 0.3f;
        }
    }
    for (int cur = m_effector; cur >= 0 && cur != m_pelvis;
         cur = m_parents[static_cast<std::size_t>(cur)]) {
        m_settlePriorWeights[static_cast<std::size_t>(cur)] = kPosePriorWeight * 0.3f;
    }
    // The ROOT stays supple too: planting flexibility comes almost entirely from the pelvis
    // giving a couple of centimeters, and the pose CONTRACT the settle protects is the held
    // effector plus the limbs' shapes (their own priors), not the root's absolute position — a
    // stiff root prior here left strained releases stalled with the feet hovering just off
    // their pins.
    if (m_pelvis >= 0) {
        m_settlePriorWeights[static_cast<std::size_t>(m_pelvis)] = kPosePriorWeight * 0.3f;
    }
}

bool IkRig::settleToPins(std::vector<glm::vec3>& positions,
                         const std::vector<glm::quat>& frameSeed) {
    const int n = m_graph.nodeCount();
    if (!dragActive() || m_pins.empty() || static_cast<int>(positions.size()) != n ||
        static_cast<int>(frameSeed.size()) != n ||
        static_cast<int>(m_settlePrior.size()) != n) {
        return false;
    }
    // The pins land; the RELEASED EFFECTOR is held as a pin at its mouse-up position ("the pose
    // must hold when letting go" — the user placed that joint there deliberately); everything
    // else eases toward the release-pose prior. The drag's active set (paths joining effector,
    // pins, and root) is exactly the settle's — same targets — so it is reused rather than
    // re-marked every tick; the rest of the body keeps its local pose and rides along.
    std::vector<IkEffector> effectors = m_pins;
    bool effectorPinned = m_effector < 0;
    for (const IkEffector& pin : m_pins) {
        effectorPinned = effectorPinned || pin.node == m_effector;
    }
    // The released joint is held only for LIMB gestures (a placed hand or foot). After a TRUNK
    // drag (chest, hip, head — effectorIsTrunk) the body must be free to drop onto its feet:
    // with the chest pinned the settle's first round could only make the worst foot pin WORSE
    // (the body cannot descend 4cm around a fixed chest), was reverted, and the figure stayed
    // standing 4cm in the air after every stepping chest drag. The release pose stays the soft
    // prior, so the trunk follows the landing rather than being yanked anywhere else.
    if (!effectorPinned && !effectorIsTrunk()) {
        effectors.push_back({m_effector, m_settleEffectorTarget, true, 1.0f});
    }
    FabrikSolver::Settings settings;
    settings.maxIterations = kIterationsPerTick;
    settings.priorIterationNorm = kPriorIterationNorm; // settle stiffness tuned at 6 iterations
    settings.floorY = m_groundOffsetY;
    settings.floorClearance = &m_floorClearance;
    FabrikSolver::solve(m_graph, m_active, effectors, m_edgeRestDir, m_edgeRestLen,
                        m_edgeConstraint, frameSeed, positions, settings, &m_settlePrior,
                        &m_settlePriorWeights);
    return true;
}

bool IkRig::solveDrag(const glm::vec3& target, std::vector<glm::vec3>& positions,
                      const std::vector<glm::quat>& frameSeed) {
    const int n = m_graph.nodeCount();
    if (!dragActive() || static_cast<int>(positions.size()) != n ||
        static_cast<int>(frameSeed.size()) != n) {
        return false;
    }

    // Deadband: nothing meaningfully to do (cursor holding still, pins already planted).
    bool satisfied =
        glm::length(positions[static_cast<std::size_t>(m_effector)] - target) <= kGoalDeadband;
    for (const IkEffector& pin : m_pins) {
        satisfied = satisfied &&
                    glm::length(positions[static_cast<std::size_t>(pin.node)] - pin.target) <=
                        kGoalDeadband;
    }
    if (satisfied) {
        // A satisfied EXPLICIT-ANCHOR (pelvis) drag may still owe a GATHERING step: the hip is
        // on its target and every pin holds, but a trailing foot can be far from its
        // target-centered stance spot — and only updateStepping can see that. Its confirm
        // counter keeps counting here; the tick a step initiates, the swing target moves and
        // the deadband stops being satisfied, so the normal solve path takes over. And for ANY
        // drag, a step already IN FLIGHT must keep advancing: the deadband can become satisfied
        // mid-swing (the foot converged onto the frozen mid-arc target while the cursor held
        // still), and returning without the advance left the foot hanging mid-air forever.
        if (!m_pins.empty() && !m_suspended) {
            if (m_stepPin >= 0) {
                updateStepping(positions, false, 0.0f, nullptr); // advance only
                return true;
            }
            if (m_effector == m_graph.root()) {
                const bool crouchIntent =
                    target.y < m_startPose[static_cast<std::size_t>(m_pelvis)].y - 0.08f;
                if (!crouchIntent) {
                    const glm::vec2 anchor(target.x, target.z);
                    updateStepping(positions, true, 0.0f, &anchor);
                    return m_stepPin >= 0;
                }
            }
        }
        return false;
    }

    // Worst steppable pin error at SOLVE ENTRY (the FK pose, before any solving) — the strain
    // half of the step trigger: an extreme drag reaches poses whose joint limits can no longer
    // hold a foot on its pin, and the foot gets dragged along the floor.
    float entrySteppablePinErr = 0.0f;
    for (std::size_t p = 0; p < m_pins.size(); ++p) {
        if (p < m_pinSteppable.size() && m_pinSteppable[p]) {
            entrySteppablePinErr = std::max(
                entrySteppablePinErr,
                glm::length(positions[static_cast<std::size_t>(m_pins[p].node)] -
                            m_pins[p].target));
        }
    }

    // Dragging the pelvis itself: the root is the one joint FBIK can place directly (root
    // translation is free), so it becomes a PINNED effector on the drag target and the solve
    // re-plants the planted feet around it — a crouch. Balance is deliberately skipped here: the
    // user is driving the pelvis explicitly.
    FabrikSolver::Settings settings;
    settings.maxIterations = kIterationsPerTick;
    settings.priorIterationNorm = kPriorIterationNorm;
    settings.minStepDisplacement = kOutputDeadband;
    settings.floorY = m_groundOffsetY;
    settings.floorClearance = &m_floorClearance;

    if (m_effector == m_graph.root()) {
        // An explicit pelvis drag is a deliberate whole-body gesture (a crouch): both legs must
        // fold in coordination every tick, which needs more solver iterations than a limb drag.
        settings.maxIterations = kIterationsPerTick * 3;
        std::vector<IkEffector> effectors = m_pins;
        // USER pins bound an explicit pelvis drag: the root target is projected into each user
        // pin's reach ball (a pinned hand holds — the hip drop stops where the arm runs out),
        // unlike ground contacts, which a pelvis drag may lift by design (the leashes are off
        // for a pinned root in the solver).
        glm::vec3 rootTarget = target;
        for (std::size_t p = 0; p < m_pins.size(); ++p) {
            const IkEffector& pin = m_pins[p];
            if (!m_pinUser[p] || pin.leashRadius <= 0.0f || pin.leashRadius > 1e8f) {
                continue;
            }
            const glm::vec3 center = pin.target - pin.leashOffset;
            const glm::vec3 d = rootTarget - center;
            const float len = glm::length(d);
            if (len > pin.leashRadius) {
                rootTarget = center + d * (pin.leashRadius / len);
            }
        }
        effectors.push_back({m_effector, rootTarget, true, 1.0f});
        FabrikSolver::solve(m_graph, m_active, effectors, m_edgeRestDir, m_edgeRestLen,
                            m_edgeConstraint, frameSeed, positions, settings, &m_startPose,
                            &m_priorWeights);
        // A LATERAL pelvis drag walks the figure: moving the hip sideways/forward drags the
        // planted feet off their pins (the strain trigger — there is no balance signal here,
        // the pelvis is user-driven), and the feet re-plant under the moved body exactly like
        // an upper-body lean. A DOWNWARD pelvis drag is a crouch and keeps its feet planted —
        // measured against the drag-START pelvis height, not per tick, so a diagonal
        // down-and-across gesture stays a crouch throughout.
        if (!m_pins.empty()) {
            const bool crouchIntent =
                target.y < m_startPose[static_cast<std::size_t>(m_pelvis)].y - 0.08f;
            const glm::vec2 anchor(target.x, target.z);
            updateStepping(positions, /*allowTrigger=*/!crouchIntent,
                           crouchIntent ? 0.0f : entrySteppablePinErr, &anchor);
        }
        return true;
    }

    // Drag intent shapes the body's compliance:
    //  - DOWNWARD intent (target below the grabbed joint): the root prior YIELDS vertically —
    //    pushing the chest (or pulling a hand) down folds the body into a crouch over the
    //    pinned, weight-bearing feet instead of fighting the standing-height anchor.
    //  - A sustained, mostly-VERTICAL pull beyond the leashed body's reach is a deliberate
    //    LIFT: enter SUSPENSION — pins release, the root rises with the drag, and the whole
    //    body (fully active, gravity-biased, prior-free) settles hanging below the grab point,
    //    limbs dangling within their joint limits.
    const glm::vec3& effPos = positions[static_cast<std::size_t>(m_effector)];
    // Decisive gestures only (-0.15): a hand pulled a few cm downward must not sag the pelvis.
    const bool downIntent = target.y < effPos.y - 0.15f;
    // The crouch DRIVE: how far below its current height the pelvis is pulled this solve — the
    // remaining downward error, capped per solve so the descent unfolds smoothly. Applied
    // through the pelvis effector below; self-limiting (error -> 0 as the push lands).
    const float rootDown =
        downIntent ? glm::clamp(effPos.y - target.y, 0.0f, 0.05f * m_sizeScale) : 0.0f;
    if (downIntent) {
        settings.rootDownYield = kRootDownYield;
    }
    // UPWARD intent: the trunk is EXCLUDED from the solve — the arm and shoulder girdle do the
    // raising against a fixed chest, and the spine simply holds its posture. Stiffening the
    // trunk's prior was not enough: the raising limb's residual error (its joints at their
    // limits) re-recruited the spine every tick and the solve found a bow/lean the user never
    // asked for — "pulling straight up bent her over". With the trunk fixed, an up-pull raises
    // the arm along its natural arc, predictably; pulls the arc cannot serve leave the hand
    // short until SUSPENSION takes over. Forward/lateral/downward intents keep full trunk
    // recruitment (leaning into a reach, crouching under a push).
    // A TRUNK effector (the head, the chest — effectorIsTrunk) is exempt: its "limb" is the
    // neck or nothing, and excluding the trunk left a 12cm head pull moving the head 7mm.
    // Pulling the head up should straighten the spine — that is the trunk's own gesture, not
    // an arm's residual recruiting it.
    const bool upIntent = target.y > effPos.y + 0.10f && !effectorIsTrunk();
    const std::vector<float>* solveWeights = &m_priorWeights;
    const std::vector<char>* solveActive = &m_active;
    if (upIntent && !m_trunkChain.empty()) {
        m_scratchWeights = m_priorWeights;
        for (const int t : m_trunkChain) {
            m_scratchWeights[static_cast<std::size_t>(t)] = std::max(
                m_scratchWeights[static_cast<std::size_t>(t)], kPosePriorWeight * kUpTrunkStiffen);
        }
        solveWeights = &m_scratchWeights;
        m_scratchActive = m_active;
        for (const int t : m_trunkChain) {
            m_scratchActive[static_cast<std::size_t>(t)] = 0; // fixed base: the trunk stays put
        }
        solveActive = &m_scratchActive;
    }
    if (!m_suspended && !m_pins.empty()) {
        // Chain length from the grab point to the root (cached at beginDrag): the body's maximum
        // reach from the (leash-bound) pelvis, and later the hang distance the root dangles at.
        const float chainLen = m_effectorChainLen;
        const glm::vec3 strain = target - effPos;
        const float strainLen = glm::length(strain);
        bool leashBound = false;
        const glm::vec3& rootPos = positions[static_cast<std::size_t>(m_pelvis)];
        // KNOWN LATENT INCONSISTENCY (deliberately kept): this test measures the ROOT against
        // the pin TARGET, but since the socket-leash round the solver's ball is centered at
        // (target - leashOffset) and confines the socket (see IkEffector::leashOffset; the
        // pelvis-drag branch above projects against the correct center). The suspension and
        // arm-raise gates were calibrated on top of this test as written — correcting it moves
        // the lift-off boundary, so it needs those two phases re-gated, not a silent fix.
        for (const IkEffector& pin : m_pins) {
            if (pin.leashRadius > 0.0f &&
                glm::length(rootPos - pin.target) > 0.96f * pin.leashRadius) {
                leashBound = true;
                break;
            }
        }
        // The lift gate is GEOMETRIC unreachability — the target farther from the leash-bound
        // root than the fully-extended chain — NOT the instantaneous drag error: the hand lags
        // the cursor by design (damped catch-up), so error-based strain read a brisk upward
        // pull as "beyond reach" while the arm was still down and hoisted the figure instead of
        // simply RAISING THE ARM overhead (which is what a reachable up-pull must do first).
        const bool beyondReach =
            glm::length(target - rootPos) > chainLen + kSuspendStrain * m_sizeScale;
        if (leashBound && beyondReach && strainLen > kSuspendStrain * m_sizeScale &&
            strain.y > kSuspendUpFraction * strainLen) {
            if (++m_suspendTicks >= kSuspendConfirmTicks) {
                m_suspended = true;
                // Ground contacts release; USER pins stay (they are explicit intent — a body
                // hanging from a pinned hand is exactly what a lift against one produces).
                {
                    std::vector<IkEffector> kept;
                    std::vector<char> keptUser;
                    for (std::size_t p = 0; p < m_pins.size(); ++p) {
                        if (m_pinUser[p]) {
                            kept.push_back(m_pins[p]);
                            keptUser.push_back(1);
                        }
                    }
                    m_pins = std::move(kept);
                    m_pinUser = std::move(keptUser);
                    m_pinFootprint.assign(m_pins.size(), {});
                    m_pinSteppable.assign(m_pins.size(), 0);
                    m_pinStanceOffset.assign(m_pins.size(), glm::vec2(0.0f));
                }
                m_supportHull.clear();
                m_stepPin = -1; // an in-flight step's foot is released with the rest
                m_suspendHang = chainLen;
                // Only real body-mass segments dangle (trunk, limbs — the balance mass model
                // already classifies them); token-mass bones (face, fingers, helpers) keep
                // their local pose and ride. Without this the JAW — a hinged joint — was
                // gravity-pulled OPEN: the figure hung with its mouth agape. The effector's own
                // chain is always active regardless of mass.
                m_active.assign(m_bodyNode.size(), 0);
                for (std::size_t i = 0; i < m_bodyNode.size(); ++i) {
                    if (m_bodyNode[i] && m_masses[i] > kRealMassThreshold) {
                        m_active[i] = 1;
                    }
                }
                for (int cur = m_effector; cur >= 0;
                     cur = m_parents[static_cast<std::size_t>(cur)]) {
                    if (!m_bodyNode[static_cast<std::size_t>(cur)]) {
                        break;
                    }
                    m_active[static_cast<std::size_t>(cur)] = 1;
                    if (cur == m_pelvis) {
                        break;
                    }
                }
            }
        } else {
            m_suspendTicks = 0;
        }
    }

    if (m_suspended) {
        settings.gravityBias = kSuspendGravity * m_sizeScale;
        settings.rootDownYield = 1.0f;
        // The root HANGS below the grab point (a soft effector at chain-length under the
        // target): without it, gravity dragged the whole body to the floor while the hand
        // reached up alone — the root must rise with the drag for the body to dangle.
        std::vector<IkEffector> effectors{
            {m_effector, target, false, 1.0f},
            {m_pelvis, glm::vec3(target.x, target.y - m_suspendHang, target.z), false, 0.5f}};
        effectors.insert(effectors.end(), m_pins.begin(), m_pins.end()); // surviving user pins
        // No pose prior: the hanging body is shaped by gravity + the joint limits alone.
        FabrikSolver::solve(m_graph, m_active, effectors, m_edgeRestDir, m_edgeRestLen,
                            m_edgeConstraint, frameSeed, positions, settings, nullptr, nullptr);
        return true;
    }

    std::vector<IkEffector> effectors;
    effectors.reserve(m_pins.size() + 2);
    effectors.push_back({m_effector, target, false, 1.0f});
    effectors.insert(effectors.end(), m_pins.begin(), m_pins.end());

    FabrikSolver::solve(m_graph, *solveActive, effectors, m_edgeRestDir, m_edgeRestLen,
                        m_edgeConstraint, frameSeed, positions, settings, &m_startPose,
                        solveWeights);

    // Auto-balance: when the solved CoM's ground projection leaves the (margin-inset) support
    // polygon, a soft pelvis effector pulls it back. The engagement machinery exists to make
    // that influence CONTINUOUS — the original design skipped the balanced re-solve whenever
    // the correction was negligible, and under a sustained pull the loop crossed that on/off
    // boundary every few ticks, toggling between two visibly different solutions: with every
    // idle limb riding on the trunk, the user saw it as TREMBLING while pulling. Three rules,
    // each verified against the tremble metrics (every cheaper variant — skip-when-quiet,
    // iteration-scaled, trust-region-scaled — put some boundary back and measurably churned):
    //  1. The re-solve runs EVERY tick, at full depth, on a COPY of the solution; the result is
    //     blended in by ENGAGEMENT (positions = mix(plain, balanced, engage)) — exactly the
    //     plain solve at engage 0, exactly the old engaged re-solve at engage 1, a plain lerp
    //     between (the transient mid-ramp blend bends bone lengths a hair; extraction re-imposes
    //     them through FK immediately).
    //  2. Engagement follows a LOW-PASSED correction (m_balanceCorrection), so tick-scale
    //     flicker of the raw need (the CoM dancing on the inset boundary) never reaches the
    //     blend.
    //  3. The effector's correction target is scaled by engagement, so the correction
    //     DECELERATES as the CoM approaches the polygon instead of overshooting deep inside —
    //     where the need reads zero, the correction would release, and the loop would swing
    //     back out (the relaxation oscillation the low-pass alone could not remove).
    // Not under UPWARD intent: the trunk (pelvis included) is excluded from the solve there, so
    // the pelvis effector is inert and the re-solve would only DOUBLE the solve depth in a
    // regime whose damped-motion behavior is tuned without it (measurably worsening the
    // up-sweep's transient reversal at the suspension boundary). The up-intent flip is already
    // a binary structure change in the design (the active set restructures), so this skip adds
    // no new boundary class.
    if (!m_supportHull.empty() && m_pelvis >= 0 && m_pelvis != m_effector && !upIntent) {
        glm::vec2 need(0.0f);
        BalanceController::balanceCorrection(positions, m_parents, m_masses, m_supportHull,
                                             kBalanceMargin * m_sizeScale,
                                             need); // false leaves need at 0
        m_balanceCorrection = glm::mix(m_balanceCorrection, need, kBalanceSmoothing);
        const float engage =
            (rootDown > 0.005f || m_healing)
                ? 1.0f
                : glm::clamp(glm::length(m_balanceCorrection) / (kBalanceRamp * m_sizeScale),
                             0.0f, 1.0f);
        static const bool kBalTrace = std::getenv("IK_BAL_TRACE") != nullptr;
        if (kBalTrace) {
            std::fprintf(stderr, "[bal] need=%.4f corr=%.4f engage=%.3f down=%.3f heal=%d\n",
                         glm::length(need), glm::length(m_balanceCorrection), engage, rootDown,
                         m_healing ? 1 : 0);
        }
        if (engage > 0.0f) {
            // XZ from the engagement-scaled smoothed correction; Y from the PRIOR height —
            // EXCEPT under downward intent, where the effector actively DRIVES the pelvis down
            // by the remaining descent (capped per solve): pushing the chest down folds the
            // body into a crouch over the pinned feet (the passive yield alone was not enough —
            // the backward pass kept re-imposing the standing configuration from the un-driven
            // root). The prior-height default is what lets a hovering figure keep descending
            // while balance fires (the engaged effector makes the pelvis prior-exempt, so a
            // current-height Y silently disabled the root prior).
            const float pelvisY =
                downIntent ? positions[static_cast<std::size_t>(m_pelvis)].y - rootDown
                           : m_startPose[static_cast<std::size_t>(m_pelvis)].y;
            const glm::vec2 corr = m_balanceCorrection * engage;
            const glm::vec3 pelvisTarget =
                glm::vec3(positions[static_cast<std::size_t>(m_pelvis)].x + corr.x, pelvisY,
                          positions[static_cast<std::size_t>(m_pelvis)].z + corr.y);
            std::vector<IkEffector> balanced = effectors;
            balanced.push_back({m_pelvis, pelvisTarget, false, kPelvisWeight});
            m_balancedScratch = positions;
            FabrikSolver::solve(m_graph, *solveActive, balanced, m_edgeRestDir, m_edgeRestLen,
                                m_edgeConstraint, frameSeed, m_balancedScratch, settings,
                                &m_startPose, solveWeights);
            if (engage >= 1.0f) {
                positions = m_balancedScratch;
            } else {
                for (std::size_t i = 0; i < positions.size(); ++i) {
                    positions[i] = glm::mix(positions[i], m_balancedScratch[i], engage);
                }
            }
        }
    }
    // Balance-driven foot re-planting: advance an in-flight step (always — a swing must land
    // whatever the drag does meanwhile), or evaluate the step trigger. New steps only while
    // grounded and outside the special regimes: upward intent barely moves the CoM, a healing
    // drag is already descending onto its pins, and a downward crouch briefly drags feet along
    // the floor by design (its transient pin error is not a step signal).
    if (!m_suspended && !m_pins.empty()) {
        updateStepping(positions, /*allowTrigger=*/!upIntent && !m_healing,
                       downIntent ? 0.0f : entrySteppablePinErr, nullptr);
    }
    return true;
}

} // namespace pose
