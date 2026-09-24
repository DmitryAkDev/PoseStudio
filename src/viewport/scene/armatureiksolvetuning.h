/**
 * @file armatureiksolvetuning.h
 * @brief Every tuning constant of the Armature's per-tick IK solve (armatureiksolve*.cpp), with
 *        the reasoning each was set by, and the env probes (A/B levers) more than one TU reads.
 *
 * The numbers are the model: weights per m^2 / rad^2, stiffness classes, the shares and ranges
 * every gesture is gated by. Each is annotated with what it was measured against (the IK harness
 * on the eight reference rigs, the character sweep, the in-app galleries) — change one, and the
 * comment says which phases will tell you what it cost. Private to the solve: only
 * armatureiksolve*.cpp and armatureikpost.cpp include it. Qt-free (std + GLM).
 */

#ifndef ARMATURE_IK_SOLVE_TUNING_H
#define ARMATURE_IK_SOLVE_TUNING_H

#include "armature.h"

#include <cstdlib>

namespace pose {

/// The cursor row: per m^2 — hard while the cursor is REACHABLE (met to a few hundredths of a
/// millimetre) — with a bounded PULL: past a miss of pull / (2 x weight) the row's cost grows
/// linearly, so its force on the body stops at kEffectorPull however far the cursor goes. That
/// force is the arbiter of how far the body follows a cursor it cannot reach: a channel gives
/// until its posture gradient (2 x stiffness x deflection / lever) matches it — the supple
/// dragged limb entirely, each stiff trunk joint some ten degrees, the pelvis under a
/// centimetre, the centre of mass a centimetre past its support — and no further: the hand
/// lets the cursor go instead of folding the figure over after it (at 1e4 an out-of-reach
/// forward pull leaned the body to its joint limits and stepped it; a head pulled up 15cm
/// walked).
constexpr double kEffectorWeight = 1.0e6;
constexpr double kEffectorPull = 200.0;
constexpr double kRootDragPullScale = 5.0;
/// The suspended root's hang below the cursor (see the SUSPENDED note in solveIk).
constexpr double kSuspendHangWeight = 1.0e4;
/// The solve root's ROTATION under a pelvis drag, per rad^2 (see rootTurns). 80, from 20: at 20
/// a kneel pitched the whole torso 56 degrees forward and splayed the thighs 29 (rotating the
/// pelvis was cheaper than anything the legs could do), the head at 0.81m; at 80 it pitches 14,
/// the thighs stay in line, the head at 1.07m, and the sit, the all-fours and the deep crouch
/// keep what they need of it. (80 had been tried and rejected before — while the foot's
/// orientation rows were mis-linearized at a large pitch (jointsolver.cpp: lateralAxisError) and
/// every kneel crawled through a different path; that verdict was the solver's, not the number's.)
constexpr double kRootRotationStiffness = 80.0;
/// Pins are harder still: the saturated pull moves a pin by pull / (2 x pinWeight), a quarter
/// of a micron.
/// A user pin outranks a contact sharing its chain.
constexpr double kPinWeight = 1.0e8;
constexpr double kUserPinScale = 4.0;
/// A pin's held orientation, in metres per radian against its position rows: PLACE wins a
/// limit-forced compromise (1 degree weighs like 3.5mm).
constexpr double kPinOrientScale = 0.2;
/// Balance: the centre of mass kept over the support. Its pull grows with the distance outside
/// the (inset) polygon and meets the cursor's bounded pull at a centimetre (pull / (2 x weight)):
/// an out-of-reach hand pull leans the body to the edge of its support and no further (at 2e3
/// the lean ran 5cm past it and the balance stepper answered a plain forward hand pull with a
/// step). Steps are for where the user takes the BODY — a pelvis or trunk drag's own trigger.
constexpr double kBalanceWeight = 1.0e4;
/// The balance requirement ramps in at this pace after the support changes (see
/// m_jsBalanceSlack).
constexpr float kBalanceRampStep = 0.01f;
/// ... and a support that GROWS is grown into at this pace, metres a tick (see m_jsGrowFrom in
/// solveIk): the slack's own, so a landing gives a lean back no faster than a lift-off asks for one.
constexpr float kSupportGrowStep = 0.01f;
/// The posture stiffness classes (see the file note) and the range normalization's bounds: a
/// channel narrower than kMinRangeDeg is priced as that (a +-5 degree knee play is not 1000x a
/// shoulder), an unlimited one as kFreeRangeDeg.
constexpr double kLimbStiffness = 1.0;
constexpr double kPinLimbStiffness = 0.25;
constexpr double kTrunkStiffness = 40.0;
constexpr double kPinLimbTwistScale = 256.0;
constexpr double kRootStiffness = 4500.0; // per m^2: a hand pull barely moves the pelvis (under 1cm on every rig)
/// The pelvis gives VERTICALLY far more readily than sideways: a push down on the chest or a
/// hand folds the knees (a crouch), where a sideways pull must not slide the hips.
constexpr double kRootStiffnessVertical = 1000.0;
/// ... and gives WAY under a push DOWN. The vertical price above meets the cursor's bounded pull
/// at pull / (2 x 1000) = 10cm, so however far the chest (or a hand) was pushed down, the hips
/// sank 10cm and stopped — "she crouches a bit and gets stuck". It cannot simply be cheap (at
/// 300 a gentle SIDEWAYS pull sank the hips 2cm to lengthen its reach), so it yields with the
/// GESTURE: as the target goes below where the drag began (from 5cm, fully by 20cm,
/// figure-scaled) and mostly vertically (its downward share of the whole displacement from 60%,
/// fully by 90%), the price blends geometrically down to this — a smooth function of the
/// target alone, so the pose stays a function of the target (no intent flipping on and off
/// under a held cursor) and bringing the cursor back up stands her back up.
///
/// The same going UP, for a drag that BEGINS crouched: let go of in a crouch, the crouch is the
/// next drag's posture reference, and pulling the chest (or the head, or a hand past the arm's
/// reach) back up met the same cap from below — the hips rose 10cm and stopped, the spine
/// stretching after the cursor ("pulling back up does not cause her to stand back up"). So the
/// price yields upward too, by the same travel and share, scaled by the HEIGHT THE LEGS HAD
/// LEFT when the drag began (m_jsRootRiseRoom: nothing under 2cm, fully from 8cm,
/// figure-scaled). A drag that begins standing has none, so its upward pulls are untouched —
/// an arm raise must not buy tiptoe with a cheap pelvis — and the room is a constant of the
/// drag, so the pose is still a function of the target.
///
/// RISING also yields the pelvis's HORIZONTAL price (m_jsRootRise), by the same factor: getting
/// up out of a KNEEL (the hips a third of a metre ahead of the feet) or a SIT (half a metre
/// behind them) carries the hips back over the feet, and the horizontal price meets the pull
/// at 200 / (2 x 4500) = 2cm — pulled up by the chest or the head, a kneeling figure did not
/// rise at all. Balance (its own row) still says how far the hips may go.
constexpr double kRootYieldStiffness = 30.0;
/// The pelvis's price, every way, while a dragged FOOT is on the floor (the SLIDE, see solveIk):
/// per m^2. A stance cannot widen or lengthen without the hips coming between the feet and down —
/// that is geometry, not preference (a hand pull is the opposite case: there the pelvis must
/// stay put, which is what the prices above are for). At those prices a foot dragged 55cm forward
/// stopped 4cm short, 40cm sideways 7cm short, the hips having followed 2cm: a single-leg squat
/// over the standing foot instead of a stride. (What takes the hips between the feet is their
/// REFERENCE, which goes with the stance's centre; the price only says how far they may stray
/// from it. At 60 they strayed: crossing one leg in front of the other, the oldest generations
/// resolved the legs' volumes by carrying the hips 17cm forward. 250 halves that, and a stride,
/// a back step and a side step are the same to the millimetre.)
constexpr double kLegDragRootStiffness = 250.0;
/// A sliding foot's SOLE: pitch, heading, level (per rad^2, x the slide). Soft next to the
/// cursor's pull (200), two decades over the foot's own posture term — the sole stays on the floor
/// as the leg swings out under it, and gives way before the cursor does.
constexpr double kSlideSolePitchWeight = 20.0;
constexpr double kSlideSoleHeadingWeight = 200.0;
constexpr double kSlideSoleRollWeight = 20.0;
/// A KNEE dragged over its planted foot (see the knee drag in solveIk): the foot lets go as the
/// knee's target is pulled UP past where the drag began — from kKneeLiftFrom, gone by
/// kKneeLiftFull (m, figure-scaled) — its rows falling by kKneeLiftDecades on the way (a pin is
/// 1e8: a linear fade does nothing, see the live contacts'); and a leg that hangs from a dragged
/// knee hangs its ANKLE under the knee (per m^2, in the floor's plane, offset as the drag found
/// it: two decades over the knee's own posture term, nothing beside the cursor's pull), so a knee
/// raise bends the knee and the shin hangs, where riding the thigh rigidly it kicked forward like
/// a goose step. (A POSITION row, not the shin's world orientation held: a trailing shin is 30-40
/// degrees off that, where a uniform orientation row's small-error Jacobian is a third wrong —
/// IK_JS_LM_TRACE showed the wall of REJECTs, three ticks of the peel did not converge, and the
/// toes jumped 10cm on the tick that did.)
constexpr float  kKneeLiftFrom = 0.01f;
constexpr float  kKneeLiftFull = 0.08f;
constexpr double kKneeLiftDecades = 8.0;
/// ... all but the ankle's PLACE on the floor's plane, which goes in two stages: hard to a soft
/// spring (1e8 to 1e2: softer than the pelvis's own price, or the hips share the stretch) with the rest, then the spring itself over the rest of the raise (to 1 by
/// kKneeTrailFull of lift, then gone). A lifted foot TRAILS: it comes up off its spot and swings
/// under the knee as the thigh comes up — let go of all at once it had 25cm to travel (8cm of
/// lift is 35 degrees of thigh, and a hanging shin puts the foot under the knee) while the
/// cursor moved two: a whip, 8cm a tick at the toes.
constexpr float  kKneeTrailFull = 0.35f;
/// ... and a letting-go foot's PLACE spring (its floor spot, still strong at 20cm of lift) swings
/// toward the hang's target over this much lift (m, figure-scaled), by as much as the shin the
/// drag found was NOT vertical (see the knee-drag foot's rows in ikPinRowsEnd). WITH the pin's
/// release (kKneeLiftFull), no faster and no slower: at 3cm the merged row's target was under the
/// knee while its weight was still a planted pin's (161mm yanks); at 10-15cm the foot had followed
/// its found spot UP behind the rising knee first, folded to the knee's limit, and sat in that
/// basin until it flipped 25-34cm in a tick (three rigs and two custom characters).
constexpr float  kKneeUnderLift = 0.08f;
constexpr float  kKneeUnderReachFrom = 0.25f; ///< The hang asks the ankle UNDER the knee from this share of the shin's length of reach ...
constexpr float  kKneeUnderReachTo = 0.90f;   ///< ... and the floor's reach point beyond this (blended between).
constexpr float  kKneeGoalRaiseShare = 0.30f; ///< The knee's goal rises to where a vertical shin stands on the floor, by at most this share of the shin (ramped by the reach).
constexpr float  kTwistAlongFrom = 0.8f; ///< A channel whose axis lies this far along its bone (cosine) begins to carry the twist price ...
constexpr float  kTwistAlongFull = 0.95f; ///< ... and carries it in full from this.
constexpr double kKneeTrailLevelShare = 0.05;
constexpr float  kKneelSeatUprightDeg = 30.0f; ///< A kneel-up's hips are home at the seat height once the trunk's target stands within this of straight up about it ...
constexpr float  kKneelSeatLevelDeg = 80.0f;   ///< ... and where they were found while it lies within this of level (blended between).  ///< The share of the foot's orientation holds kept once its trail is complete (the toes' 4e4 -> 2000).
/// ... pulling this much harder than the standing hang for a shin found lying flat (a kneel's).
constexpr float  kKneeSwingHangScale = 3.0f;
constexpr float  kKneeLiftEase = 0.15f; ///< a knee coming off a planted foot: its height eases in over this much lift
constexpr double kKneePlaceSpringDecades = 6.0;
constexpr double kKneeAnkleHangWeight = 100.0;
/// Under a PELVIS drag the body still BALANCES — with its trunk: the hips are where the cursor
/// puts them, so what takes the centre of mass back over the feet is the spine and the pelvis's
/// own pitch (a hip HINGE: hips back, chest forward). It was simply off there, and a figure whose
/// hips were taken 10cm back stood bolt upright with her head 7cm behind her heels; a half squat
/// sat back the same way. It comes in with the pelvis's HEIGHT (the cursor's, as a share of the
/// standing height: none under kPelvisBalanceFrom, in full from kPelvisBalanceFull): it is the
/// standing body's — a hinge, a half squat. Lower than that the hips are on their way to the
/// floor, where they are sat on, not balanced over (at 0.30-0.50 one generation's kneel still
/// caught it on the way down and jumped 3cm in its still hold).
constexpr float kPelvisBalanceFrom = 0.70f;
constexpr float kPelvisBalanceFull = 0.85f;
constexpr float kPelvisBalanceDownFrom = 0.55f;
constexpr float kPelvisBalanceDownGone = 0.80f;
constexpr float kTrunkFollowDownFrom = 0.30f; ///< The trunk follow stands down for a drag whose downward share of travel is over this ...
constexpr float kTrunkFollowDownGone = 0.55f; ///< ... and is gone from this (a bow is a fold, not a walk).
constexpr float kUnfoldFromDeg = 10.0f; ///< A trunk found tilted more than this that comes upright brings its hips under its target ...
constexpr float kUnfoldFullDeg = 25.0f; ///< ... in full from this much of found tilt.
/// A TRUNK joint dragged ACROSS the floor (the chest, the neck, the head) takes the BODY with it:
/// the first of the travel is a LEAN (the spine's), and past it the hips FOLLOW — their
/// REFERENCE moves, which is the only way a body part goes anywhere (a price can only prefer
/// less travel). The lean's share is sized by the grab's height over the hips, its lever: none
/// of the travel follows under kTrunkLeanFree of it, and past that the lean saturates at
/// kTrunkLeanWidth more (travel - w tanh(travel / w)). Left to the prices, the hips stood
/// still (4500/m^2 against the cursor's bounded pull is 2cm) and BALANCE walled the lean in: a
/// chest pulled 10cm back stopped 5cm short, 20cm back 12 short with the lower spine at its
/// limits and the upper curled forward to counterweight it, 20cm forward 5 short — and no
/// step ever answered, the rig's trigger for trunk drags being a miss of 12cm.
constexpr float kTrunkLeanFree = 0.10f;
constexpr float kTrunkLeanWidth = 0.20f;
/// ... and no more of a lean than BALANCE has room for: a lean of the grabbed joint by l carries
/// the centre of mass l x (the upper body's mass moment over the hips / the body's mass x the
/// lever), and the support has only so much room the way the drag goes — 4cm behind a standing
/// figure's weight, 10 ahead of it. Past this share of that room the hips follow instead (a
/// neck pulled 20cm back leaned into the support's edge and stopped 27mm short, the follow
/// still waiting for a lean that could not come).
constexpr float kTrunkLeanBalanced = 0.8f;
/// The sway's roll answers a leg that has RUN OUT (see the sway in solveIk): in full while the
/// shortest-handed standing leg has under kSwayLegInHandFrom of length in hand where the cursor
/// takes the hips, none from kSwayLegInHandGone.
constexpr float kSwayLegInHandFrom = 0.03f;
constexpr float kSwayLegInHandGone = 0.10f;
/// The pelvis's rotation price is its PITCH's (kRootRotationStiffness); its heading and its roll
/// cost this many times as much (see the root's channels in solveIk).
constexpr double kRootSquareness = 5.0;
/// The spine's neighbours are held to the same SHARE of their ranges (see the couplings in
/// solveIk), per unit of share^2 — the order of a spine channel's own posture price in those
/// units (40), so an S-curve of half a range each way costs as much as the bends themselves.
/// A knee coming down over its planted foot holds its PLACE this softly (per m^2) until it has
/// landed (solveIk's hard live pins); its height is held in full.
constexpr double kKneeLandingHold = 1.0e4;
/// With her knees planted (hard live pins) a pelvis drag pays this many times the root's
/// rotation price for the pelvis's tilt (solveIk).
constexpr double kKneelTiltScale = 5.0;
/// The foot below a knee that is on the floor holds its PLACE this softly (per m^2): solveIk's
/// ball row.
constexpr double kKneelFootHold = 1.0e4;
/// ... and it is let go of altogether as the HIPS go ahead of that knee under a pelvis drag (a body
/// lowered onto its belly folds its shins up behind it): the foot's rows fade over this travel.
constexpr float kProneFootFreeFrom = 0.08f;
constexpr float kProneFootFreeFull = 0.25f;
/// ... and a planted knee is unloaded by a RISE only once the hips stand this share of the thigh's
/// length over it (getting up off the belly comes onto all fours first).
constexpr float kKneeUnloadHeight = 0.6f;
constexpr double kSpineCoupling = 80.0;
/// ... and the NECK's links to it (see the couplings in solveIk): the neck is the spine's top.
constexpr double kNeckCoupling = 80.0;
/// The pelvis's horizontal price under a trunk drag whose stance cannot walk and whose lean has
/// run out of balance room (see m_jsTrunkCounter in solveIk): low enough that the balance row
/// and the cursor place the hips between them.
constexpr double kTrunkCounterStiffness = 250.0;
/// The spine's twist and side-bend channels under a trunk drag that stays in the sagittal plane
/// (see sagittalLock in solveIk): this many times their price.
constexpr double kSagittalLock = 20.0;
/// The bow's arc is a drag's that goes FORWARD: none of it under this cosine of the travel's angle
/// off the figure's heading, all of it from the second.
constexpr float kBowArcSagittalFrom = 0.70f;
constexpr float kBowArcSagittalFull = 0.92f;
/// How much of a rigid trunk's arc (see leanArc in solveIk) a spine that CURLS as it folds reaches.
constexpr double kBowArcShare = 0.75;
/// The pelvis pitch's nominal RANGE, degrees: what its share of the bend is measured in where it is
/// coupled to the first spine bone's (see kSpineCoupling) — a hip folds about 90 degrees.
constexpr double kTrunkHingeRangeDeg = 90.0;
/// ... and ON HER KNEES (m_jsKneelStart) the fold is the HIPS': the pelvis's pitch costs this, and
/// counts as this many degrees of range in the spine's coupling chain. Standing, a bow is shared
/// between the hips and a curling spine, and looks it; over planted knees the same sharing took a
/// figure going onto all fours into a cat stretch - the spine at its limits under a pelvis pitched
/// 50 degrees, the head tucked under between her arms - where a body folds 90 degrees at the hips
/// under a back that stays all but flat, its thighs upright over its knees.
constexpr double kKneelFoldStiffness = 5.0;
/// A HAND ON THE FLOOR lies on its PALM (see the live contacts in solveIk): the soft pulls, per
/// rad^2, that roll it flat about its fingertip (pitch, and level about the fingers' line) and
/// keep the fingers' heading. Soft beside everything that places the arm: a wrist a hand's length
/// over the floor cannot lie flat, and must not drag the shoulder down to try.
constexpr double kPalmPitchWeight = 30.0;
constexpr double kPalmLevelWeight = 30.0;
constexpr double kPalmHeadingWeight = 3.0;
/// ... and the palm's TARGET turns from the rotation the hand landed with toward flat at this pace,
/// degrees a tick: the pull is soft, but so is a wrist, and asked for flat at once a hand that
/// landed on its fingertips snapped down 16cm in the tick after.
constexpr float kPalmTurnStepDeg = 4.0f;
/// ... and the hold of the FINGERTIP's height on the floor, per m^2: the contact's own, faded as
/// the contact is. (Softened to 2000 for a day so that a figure taken back up off all fours in the
/// same drag would let go of the floor - which was the contact's UNLOAD being read over the height
/// the hand touched at, not the height it lies at: see the settle in IkRig::updateContacts and
/// m_jsContactRise. Soft, it let the hands slide and cross under a body coming down: the male
/// rig's landed 20cm lopsided, the chest 16cm off its cursor.)
/// A hand contact's arm is SLACK — the hand held down, a ceiling on it — while the wrist's reach from
/// its limb junction is under this share of the arm's span; above it the rise unloads the hand. Kept
/// ABOVE the rig's lift-off test (kLiveContactTaut 0.95), so a hand hanging from its straight arm
/// reads taut there every tick and lifts off (solveIk, the live-contact block).
constexpr float kHandSlackShare = 0.97f;
/// A kneel-start trunk drag unloads its HAND contacts over this much upward travel of the target
/// (metres, figure-scaled): the trunk leaving its hands, which come off the floor at once.
constexpr float kKneelUpUnloadCm = 0.015f;
constexpr double kFingertipHold = 1.0e8;
/// A HANGING arm's elbow SOFTENS as its hand nears the floor (hangIdleArms): this much flexion,
/// eased in over the fingertip's last kArmLandSoftenFrom of height, at kArmLandSoftenStepDeg a
/// tick. An arm that lands dead straight with its hand straight below it is a fully extended
/// chain - the straight limb's saddle: shortening it is invisible to a first-order model - and
/// the tick such hands touched down the chest lost its cursor by 3-5cm, the solver's fold retry
/// kicked both elbows, and the pose hopped to the basin it found: the trunk 13 degrees back up,
/// 15cm at the brow. A relaxed arm is not a ramrod either.
constexpr float kArmLandSoftenDeg = 10.0f;
constexpr float kArmLandSoftenFrom = 0.15f;
constexpr float kArmLandSoftenStepDeg = 1.5f;
constexpr double kKneelFoldRangeDeg = 300.0;
/// THE SEAT ROLLS (solveIk): the share of the target's swept angle about the seat that the
/// pelvis's pitch reference takes, and the most it takes either way (degrees).
constexpr double kSeatRollShare = 1.0;
constexpr double kSeatRollMaxDeg = 110.0;
/// THE BOW: under an upper-body trunk drag that began standing the pelvis's PITCH joins the
/// unknowns, per rad^2 (see m_jsTrunkHinge in solveIk).
constexpr double kTrunkHingeStiffness = 20.0;
/// The hinge's release once a pelvis drag WALKS: slack per tick, and the slack at which the row is
/// dropped for the rest of the drag (no imbalance a standing figure can have is that large).
constexpr float kPelvisBalanceReleaseStep = 0.005f;
constexpr float kPelvisBalanceReleaseGone = 0.40f;
/// ... and as the hips LEAVE THE STANCE (their horizontal distance from where the stance, as the
/// rig has walked it, puts them): past kPelvisBalanceStanceFrom the row's slack grows
/// kPelvisBalanceStanceGain for every metre further — the imbalance grows one for one, so the
/// lean is given back at half the hips' pace and is gone 12cm on. (As a WEIGHT, fading over
/// 12-18cm, it did nothing for nine tenths of its range and let the whole lean go in the last:
/// a hard row's pose barely reads its weight until the posture terms rival it.)
constexpr float kPelvisBalanceStanceFrom = 0.12f;
constexpr float kPelvisBalanceStanceGain = 1.5f;
/// Under a hip sway (see the sway's roll in solveIk) the CHEST stays LEVEL: its roll about the
/// figure's fore axis held as the drag found it, per rad^2 — against four or five spine joints
/// at the trunk's price (~150 each) that is 97% of the pelvis's roll taken up in the spine.
/// Riding the rolled pelvis rigidly, the whole torso leaned away from the sway like a mast: 8
/// degrees for 8cm, the head back over where the hips had been.
constexpr double kSwayChestLevelWeight = 1000.0;
/// A dragged ELBOW's hand tends to STAY where it is (see the elbow drag in solveIk): its wrist held
/// at its place as the drag found it, per m^2 — two decades over the arm's own posture terms, so
/// the elbow swivels about a hand that stays to the millimetre where the arm's geometry lets it
/// (elbows out, in, tucked: what an elbow is grabbed FOR), and well under the cursor's pull
/// (200 / (2 x 300) = 33cm of stretch before the cursor gives any ground), so the hand follows
/// wherever the forearm's length says it must. It goes as the elbow's target leaves the forearm's
/// length of that place (from kElbowHoldFrom, gone by kElbowHoldGone): an elbow taken right away
/// carries its arm, as it always did.
constexpr double kElbowHandHoldWeight = 300.0;
constexpr float  kElbowHoldFrom = 0.10f;
constexpr float  kElbowHoldGone = 0.30f;
/// ... and it comes WITH THE ARM'S BEND as the drag found it (degrees of elbow flexion: none under
/// kElbowHoldBendFrom, in full from kElbowHoldBendFull): a STRAIGHT arm has no swivel — its elbow
/// lies on the line from shoulder to hand, and cannot leave it with both ends where they are —
/// so there the elbow carries its arm, as it always did. (Held, a straight arm's elbow dragged
/// 10cm inward went nowhere — the fold is bounded at straight — and 15cm forward ran the collar
/// to two of its limits and the shoulder's twist to a third, the girdle making the room the arm
/// could not.)
constexpr float  kElbowHoldBendFrom = 10.0f;
constexpr float  kElbowHoldBendFull = 40.0f;
constexpr float  kElbowReachEase = 0.10f; ///< a target inside the upper arm's reach: its depth eases in over this much
constexpr float  kElbowReachBand = 0.06f; ///< a target this far beyond it still comes back onto it
/// Under an elbow drag the shoulder GIRDLE (what lies between the arm's socket and the chest: the
/// collar) costs this many times its range-normalized price: an elbow is placed by the SHOULDER.
/// A collar's range is a third of a shoulder's, but so is its lever's reach — at its plain price
/// it did a third of every elbow move, carrying the socket 6cm as an elbow swung 8.
constexpr double kElbowGirdleStiffness = 20.0;
/// ... and the planted foot below a dragged knee may TURN on its spot (its heading, per rad^2:
/// every other planted foot's is 4e6): a knee swung out or in over a foot that must keep pointing
/// ahead is a knee that cannot go — the leg turns out from the hip, foot and all, or not at all.
/// Held hard, a knee dragged 15cm outward went 4cm, the shin's side bend and the ankle's turn at
/// their limits.
constexpr double kKneeFootHeadingWeight = 5.0;
/// What a planted leg's channels gain, per rad^2 at the full rise, toward the rest stance (see
/// the RISING note at the unknowns). A TIE-BREAKER, and it must stay one: at 2 the legs
/// straightened eagerly enough to push the hips up and forward ahead of the chest the user held,
/// the torso was crushed between them and the spine arched to its limits and twisted; the final
/// stance is as clean at 0.02 as at 2 (the upright pelvis is what makes it).
constexpr double kRiseHomeStiffness = 0.2;
/// The slack-hand CEILING (a hand held DOWN at its planted height while its arm has length to
/// give: a body pushing up off its belly keeps its hands on the floor as the arms straighten)
/// stands down for a body COMING DOWN — over this much of the target's downward travel since
/// the drag began (m, figure-scaled). See the live-contact block in solveIk.
constexpr float  kHandCeilingDownFrom = 0.02f;
constexpr float  kHandCeilingDownFull = 0.08f;
constexpr float  kRootYieldDownFrom = 0.05f;
constexpr float  kRootYieldDownFull = 0.20f;
constexpr float  kRootYieldShareFrom = 0.60f;
constexpr float  kRootYieldShareFull = 0.90f;
constexpr float  kRootYieldRoomFrom = 0.02f;
constexpr float  kRootYieldRoomFull = 0.08f;
constexpr double kMinRangeDeg = 20.0;
constexpr double kFreeRangeDeg = 360.0;
/// The floor's and the body volumes' one-sided rows: harder than the cursor (its saturated pull
/// sinks a joint 0.5mm into a volume), softer than a pin. Rows are raised for joints within
/// kPlaneMargin of a surface, so a trial step INTO it is seen by the cost that judges the step.
constexpr double kPlaneWeight = 1.0e7;
constexpr double kPlaneMargin = 0.02;
/// THE FOOT CONTACT MODEL. A standing foot is held where a foot really bears: at the BALL (the
/// first floor-level joint under the ankle), hard; the HEEL (the ankle, the rig's pin) only
/// softly at its spot and never below it (the floor row); the sole's yaw and roll hard, its
/// pitch about the ball nearly free; the toes' own orientation held, so they stay flat as the
/// foot pitches over them. A heel lift is then not a policy but the cheapest pose when the
/// ankle's dorsiflexion runs out under a deep crouch or a kneel — or the legs' length under a
/// high reach (tiptoe) — and costs kHeelHoldWeight x rise^2 against the cursor's bounded pull:
/// 200 / (2 x 2000) allows a hand or head pull 5cm of it, a pelvis drag's stronger pull the
/// ankle's whole range, and nothing lifts while the pose has any cheaper way.
constexpr double kHeelHoldWeight = 2000.0;
constexpr double kSolePitchWeight = 20.0;
constexpr double kToeHoldWeight = 4.0e4;
constexpr double kSoleRollWeight = 5.0;
/// The ball is looked for among the pin's descendants whose bind height is under this fraction
/// of the pin's own (an ankle 8cm up: under 3.6cm) and which carry children (not a toe tip).
constexpr float kBallHeightFraction = 0.45f;
/// A FOLD channel: at least this much range, all but a sliver of it to one side of straight.
constexpr float kFoldRangeDeg = 100.0f;
constexpr float kFoldNarrowFraction = 0.2f;
/// Channels with under this range are locked (a twist bone's swing axes): not unknowns.
constexpr float kLockedRangeDeg = 2.0f;
/// A pin target the rig MOVED (ground healing, a heel lift, a landing, a slide) is approached
/// at this pace rather than jumped to; faster than the stepper's own 1.5cm/tick swing.
constexpr float kPinEaseStep = 0.025f;
/// A limb asks the BODY up only by what lies beyond this share of its reach (see the rise in
/// dragIkTick): a straight arm is not a comfortable one, so a little short of the whole.
constexpr float kLimbReachShare = 0.95f;
/// ... and the pace a LIFTED body comes up to its cursor at (see dragIkTick): per tick.
constexpr float kLiftRiseStep = 0.025f;
inline const bool kNoLiftEase = std::getenv("IK_JS_NO_LIFT_EASE") != nullptr; // A/B probe
/// A cursor jump is solved through intermediate targets this far apart (continuation keeps
/// the solve on its branch; it costs iterations, never lag).
constexpr float kTargetSubstep = 0.04f;

template <typename T> T envOr(const char* name, T fallback) {
    const char* env = std::getenv(name);
    return env != nullptr ? static_cast<T>(std::atof(env)) : fallback;
}

// Env probes read by more than one stage of the solve (a stage-local static would be one stage's).
inline const double kRootRot = envOr("IK_JS_ROOTROT", kRootRotationStiffness);
inline const bool kHangAsFound = std::getenv("IK_JS_KNEE_HANG_AS_FOUND") != nullptr;   // A/B probe: the knee hang's found offset
inline const bool kNoUnderEase = std::getenv("IK_JS_KNEE_UNDER_NO_EASE") != nullptr;  // A/B probe: the hanging foot's target unEASED

/// Per damping level: the posture easing kappa (each tick the redundant posture covers
/// 1 / (1 + kappa) of what separates it from its solution: 95% in 0 / 4 / 7 / 13 ticks) and the
/// target follower's natural frequency in rad/s (critically damped; it trails a steadily moving
/// cursor by 2v/omega — one tick of cursor travel at 120 — and 0 = no follower).
struct DampingLevel {
    double kappa;
    float  followOmega;
};
constexpr DampingLevel kDampingLevels[Armature::kIkDampingLevels] = {
    {0.0, 0.0f}, {1.0, 120.0f}, {2.0, 80.0f}, {4.0, 50.0f}};
/// The drag tick (VulkanWindow's 60 Hz timer; the harness's tick is the same unit).
constexpr float kTickSeconds = 1.0f / 60.0f;

} // namespace pose

#endif // ARMATURE_IK_SOLVE_TUNING_H
