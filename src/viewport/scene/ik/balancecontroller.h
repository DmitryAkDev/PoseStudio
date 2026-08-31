/**
 * @file balancecontroller.h
 * @brief Center-of-mass auto-balancing for the FBIK system (step 4): a mass model over the
 *        skeleton, the 2D support polygon between the planted contacts, and the pelvis
 *        correction that keeps the character balanced.
 *
 * Every bone gets an approximate segment mass (biomechanical body-segment fractions, classified
 * from bone names tolerantly across figure generations — "lThighBend" and "l_thigh" both read as
 * a thigh; same-class same-side bones split their segment's share so a Bend+Twist pair doesn't
 * double a limb's mass). The posed CoM is the mass-weighted mean of each segment's midpoint. The
 * support polygon is the convex hull, in the ground (XZ) plane, of the planted ground contacts —
 * both feet when standing, one foot mid-step, knees+feet when kneeling. When the CoM's ground
 * projection leaves the (margin-inset) polygon, balanceCorrection returns the XZ shift that
 * brings it to the nearest balanced point; the IK rig applies that shift as a soft pelvis
 * effector and re-solves — bending knees/ankles naturally, since the feet stay pinned.
 * Qt-free (std + GLM).
 */

#ifndef BALANCECONTROLLER_H
#define BALANCECONTROLLER_H

#include <glm/glm.hpp>

#include <string>
#include <vector>

namespace pose {

/**
 * @class BalanceController
 * @brief Stateless CoM / support-polygon math for the IK rig's balance loop.
 */
class BalanceController {
public:
    /// Per-bone segment masses (arbitrary consistent units — the CoM is a ratio) from tolerant
    /// name classification, split among same-class same-side bones. Unclassified bones (face,
    /// fingers, helper joints) get a token mass so they perturb, not dominate.
    static std::vector<float> assignMasses(const std::vector<std::string>& names);

    /// Mass-weighted center of the posed skeleton: each bone contributes at its segment midpoint
    /// (joint to the mean of its anatomical children; leaves contribute at the joint itself).
    static glm::vec3 centerOfMass(const std::vector<glm::vec3>& positions,
                                  const std::vector<int>& parents,
                                  const std::vector<float>& masses);

    /// Convex hull (counter-clockwise) of the contact points in the ground plane (x = world X,
    /// y = world Z). Degenerate inputs pass through (1 point / 2 collinear points stay as-is).
    static std::vector<glm::vec2> supportPolygon(std::vector<glm::vec2> contactPoints);

    /// True if @p p lies inside (or on) the counter-clockwise hull.
    static bool insidePolygon(const std::vector<glm::vec2>& hull, const glm::vec2& p);

    /// The balanced point nearest @p p: @p p itself when it lies at least @p margin inside the
    /// hull, else the closest boundary point pulled @p margin toward the hull's interior.
    /// Degenerate hulls (a single planted foot, one contact point) resolve to the point/segment.
    static glm::vec2 closestBalancedPoint(const std::vector<glm::vec2>& hull, const glm::vec2& p,
                                          float margin);

    /// One controller step: projects the posed CoM onto the ground plane and, if it isn't safely
    /// inside the support polygon, writes the XZ shift that would re-balance it to
    /// @p correctionXZ and returns true. Returns false when already balanced (or no polygon).
    static bool balanceCorrection(const std::vector<glm::vec3>& positions,
                                  const std::vector<int>& parents,
                                  const std::vector<float>& masses,
                                  const std::vector<glm::vec2>& hull, float margin,
                                  glm::vec2& correctionXZ);
};

} // namespace pose

#endif // BALANCECONTROLLER_H
