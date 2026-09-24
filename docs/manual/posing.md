# Posing

Posing in PoseStudio is direct: you grab a part of the body and move it, and the whole figure answers the way a person's body would. There is no gizmo to find and no chain to set up. This page explains the controls and what the body does with them; [Poses Step by Step](poses.md) shows how to make particular poses, and [Pose Files](pose-files.md) how to save them.

## Grabbing a joint

Move the pointer over the figure and **press the left button on the part you want to move**. PoseStudio picks the joint nearest the pointer — the wrist when you click on a hand, the knee or the hip when you click along the thigh — and tints the flesh that joint drives blue, so you always see what you have hold of. The point under your pointer is what you are holding: drag, and that point follows the cursor.

- Clicking a **joint** itself (a knee, an elbow, a wrist) takes hold of that joint.
- Clicking **along a limb** takes hold of the point you clicked: the far end of the limb segment when you click nearer it (the knee for a thigh, the hand for a forearm), the near end otherwise.
- A click on the **hips** takes hold of the pelvis; a click on the **chest** or the **head** takes hold of the trunk there.
- Joints stay clickable whether the [skeleton overlay](shading.md#the-skeleton-overlay) is on or off. Turn it on to see exactly where they are.

A left press that finds no joint orbits the camera instead, and a plain click selects or deselects the model.

## Dragging: full-body IK

**Left-drag a joint** and the body follows. The grabbed point tracks the cursor in the plane of the screen, and everything else is worked out for you, every frame:

- **The feet stay planted.** Standing feet are anchored to the floor — the ball of the foot holds hard, the heel may lift when the ankle runs out, and the toes stay flat. Nothing you drag slides a planted foot.
- **The body balances.** The weight stays over the feet: pull a hand forward and she leans; pull it further and the whole body leans as far as it can and then stops.
- **She steps when she must.** Take the hips, the chest or the head far enough across the floor and the feet step under the body, one at a time, each with a swing and a landing. A step in flight always finishes when you let go.
- **She crouches, kneels, sits and lies down.** Push the hips down and she crouches; take them down and forward and she kneels, the knees landing on the floor; down and back and she sits. Knees, hands, the seat and the head all become supports when they reach the floor and let go again when the body rises off them. See [Poses Step by Step](poses.md).
- **Joints stay within their limits.** Every joint has the anatomical range its figure was authored with — a knee bends one way, a neck turns so far — and a drag never takes a joint past it.
- **Limbs stay out of the body and out of each other.** Arms cross the chest without passing through it, a hand laid on the other forearm rests on it, crossed legs stay apart.
- **Lifting her off the floor.** Pull a hand steadily up beyond the arm's reach and, after a moment, she lifts off and hangs from it; lower her and she lands and stands back up.

The response is deliberately smooth: the figure follows the cursor with a small lag rather than snapping, and a still cursor gives a perfectly still figure. Releasing the button ends the drag; if a step or a landing was still in flight, it completes on its own, and then the drag becomes one entry in the undo history.

### Moving toward and away from the camera

The cursor moves the grabbed point across the screen. To move it **in depth** — toward or away from you — **roll the mouse wheel while the button is held**: scrolling up pushes the point away from the camera, scrolling down pulls it toward you, in steps of about 2% of its distance from the camera. The point stays under the pointer as its depth changes. Orbit to a side view to check the depth you reached.

### Special joints

Some joints have their own way of being dragged, because that is what a person means by moving them:

- **A knee** dragged swings over its planted foot — out, in, forward, back — while the foot pivots on its heel. Pull it up past what the shin allows and the foot lets go, trails, and swings in under the knee; bring it back down and it returns to its spot.
- **An elbow** dragged swivels about a hand that stays where it is (elbows out, elbows in, an arm tucked); a straight arm's elbow carries the arm.
- **A foot** on the floor dragged along it slides: the hips come between the feet and the sole stays flat. A foot lifted in the air is placed by its leg alone.
- **The hips** dragged sideways sway — the pelvis rolls and the chest stays level; dragged back they hinge so the head comes forward over the feet; dragged across the floor they walk.
- **The head** dragged takes the body with it through the neck — it leans, steps, bows and crouches. To *turn* the head, rotate it instead (below).

## Rotating a joint: FK

Sometimes you want to turn one joint and nothing else — tilt the head, cock a wrist, twist a forearm. Two ways:

- **Ctrl + left-drag** on a joint rotates that joint alone: dragging sideways turns it about its vertical axis, dragging up and down bends it about its sideways axis. Nothing else moves.
- **Hold `X`, `Y` or `Z` and roll the mouse wheel** with a joint selected: each wheel notch turns the joint 5° about that one channel of its rotation. A badge under the viewport strip shows the axis. This is the way to reach a joint's *twist*, which a drag cannot address. The hold also works in the middle of an IK drag: the wheel turns the grabbed joint while the body keeps following the cursor.

Rotations stop at the joint's limits, and a rotation that would push a limb into the body stops at the body. Each Ctrl-drag or each key-hold is one undo step.

## Joint pins

A **pin** holds a joint exactly where it is through every later drag: a hand kept on a hip, a foot that must stay on its spot, hips held on a chair.

- Select a joint (click it) and press **`P`** to pin it; press `P` again to unpin. The context menu (right-click a joint) offers **Pin Joint**, **Unpin Joint** and **Unpin All Joints**.
- Pinned joints show an **orange marker**. While you drag, the feet the body is standing on show smaller **cyan** markers — those are the drag's own contacts, not pins.
- A pin holds both the joint's place and its orientation, to microns, under any drag of another joint. Dragging the pinned joint itself moves it.
- **Pinning the hips makes a seat.** PoseStudio knows nothing about chairs, so this is how you tell it that something holds her up: sit her down by the hips, pin them, and pose the rest — balance and stepping stand down, and the pelvis rocks under a chest lean like a seated person's.
- Pins are part of the pose: they are saved in [pose files](pose-files.md), and undoing a drag restores the pins of that moment.

## The Ground button

A pose that bends the knees lifts the feet off the floor, because the hips stayed where they were. The **Ground** button on the viewport strip drops the active figure so the lowest point of its current pose rests on the floor — animated as a real fall (a metre takes about half a second). A figure sunk below the floor is lifted out at once. Grounding is refused while you are dragging, and it is not on the undo history.

Figures import standing on the floor, so you rarely need this after an IK drag: planted feet keep the figure on the ground.

## Reset and mirror

The **Edit** menu and the joint context menu offer five utilities, each one undo step:

| Utility | What it does |
| --- | --- |
| **Reset Selected Joint** | Returns the selected joint to its rest rotation. |
| **Reset Limb** | Returns the selected joint and everything below it. |
| **Reset Pose** | Returns the whole figure to its rest pose — the figure as it was imported. |
| **Mirror Pose** | Swaps the body's left and right sides: a pose that leans left leans right. |
| **Mirror Limb to Other Side** | Copies the selected joint's pose, and everything below it, to the matching joint on the other side — pose one arm, then mirror it to the other. From a centre joint such as the chest it mirrors both sides. |

Pins are left alone by all five.

## Undo and redo

**Edit → Undo** (`Ctrl+Z`) and **Redo** (`Ctrl+Y`) walk one history of everything you changed: each drag, each rotation, each pin toggle, each utility and each pose file loaded — and each lighting change on the Environment tab. An entry is recorded when a gesture ends, so a long drag is one step. Deleting an object clears the history.

## Several figures

You can import as many figures as you like. Posing addresses the **active figure** — the one whose joint you clicked last — and the utilities, the pins, the pose files and the Ground button all act on it. Click a joint on another figure to make that one active. While a model that is not a figure (an `.obj`) is the selection, the posing utilities do nothing.

## What the figure knows

Everything the body does comes from the figure you imported, not from PoseStudio's guesses:

- **Joint limits** are the ranges authored in the figure. Locked channels (a twist bone's bending axes) never move.
- **Correctives** — the sculpted shapes that fix a deep elbow or knee bend — blend live while you drag, so the mesh looks right at every frame.
- **Body proportions** set every threshold: a small character crouches, steps and kneels at her own scale.

## Tips

- Orbit to a **side view** (`3`) before pushing a figure down onto the floor or forward onto her hands — you can see the depth, and the drag plane is the one you want.
- To move a whole figure, drag the hips: they walk her across the floor.
- If a drag does not go where you expect, look at where the feet are: a planted foot is an anchor, and a pull the body cannot follow without stepping ends in a lean. Take the hips there instead, or lift a foot first.
- A figure left kneeling, sitting or lying starts the next drag from that pose — pull the hips or the chest up to get her up again.
- Reset Pose is always one step away when a pose gets away from you.
