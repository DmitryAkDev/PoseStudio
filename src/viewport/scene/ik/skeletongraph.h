/**
 * @file skeletongraph.h
 * @brief The FBIK skeleton graph: the figure's joints as an UNDIRECTED graph that can be
 *        dynamically re-rooted at any node (step 1 of the full-body IK system).
 *
 * The solver sees the skeleton as an undirected graph with a movable root: setRoot(node)
 * rebuilds the directed parent/children view and BFS traversal order the FABRIK passes walk,
 * without ever touching the engine's anatomical hierarchy (rotation extraction still happens in
 * that original hierarchy — this graph exists purely for the positional solve). The rig roots it
 * at the PELVIS, so traversal order equals hierarchy order and joint constraints are evaluated
 * in their owning parent-side frames; ground anchoring is the pinned effectors' job, not the
 * root's. (An earlier design re-rooted at a ground contact per drag — anatomically appealing,
 * but a constraint frame accumulated from the child side skews by exactly the joints' own
 * swings, so limbs "kinked" at rigid twist joints. FabrikSolver's per-edge arrays are indexed by
 * anatomical child, so only the pelvis rooting may ever reach solve() — see its header.)
 * Qt-free (std only + no GLM needed — pure topology).
 */

#ifndef SKELETONGRAPH_H
#define SKELETONGRAPH_H

#include <vector>

namespace pose {

/**
 * @class SkeletonGraph
 * @brief Undirected joint graph with a movable root: topology fixed, rooting per-solve.
 */
class SkeletonGraph {
public:
    /// Builds the undirected adjacency from the anatomical hierarchy (@p parents[i] = i's parent
    /// bone index, -1 for the skeleton root) and roots the graph at the hierarchy root initially.
    void build(const std::vector<int>& parents);

    /// Re-roots the graph at @p node: recomputes the per-node parent, children, and root-first
    /// traversal order via BFS over the undirected adjacency. Returns false if @p node is invalid.
    bool setRoot(int node);

    int root() const { return m_root; }
    int nodeCount() const { return static_cast<int>(m_adjacency.size()); }

    /// @p node's parent under the CURRENT rooting (-1 at the root). Not the anatomical parent.
    int parentOf(int node) const { return m_parent[static_cast<std::size_t>(node)]; }

    /// Root-first (BFS) traversal order under the current rooting — the order the FABRIK backward
    /// pass walks; the forward pass walks it reversed.
    const std::vector<int>& traversalOrder() const { return m_order; }

    /// Marks the union of the current-root paths of @p targets: every node on any target's walk to
    /// the root, targets and root included. The solver only moves marked nodes — everything off
    /// those paths (fingers during an arm drag, the face, …) rides along rigidly.
    std::vector<char> markActivePaths(const std::vector<int>& targets) const;

private:
    std::vector<std::vector<int>> m_adjacency; // undirected: hierarchy edges, both directions
    std::vector<int>              m_parent;    // per node, under the current rooting
    std::vector<int>              m_order;     // BFS order from the current root
    int                           m_root = -1;
};

} // namespace pose

#endif // SKELETONGRAPH_H
