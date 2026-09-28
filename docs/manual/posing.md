# Posing

Posing in PoseStudio is direct: you grab a part of the body and move it, and the whole figure answers the way a person's body would. There is no gizmo to find and no chain to set up. This page explains the controls and what the body does with them; [Poses Step by Step](poses.md) shows how to make particular poses, and [Pose Files](pose-files.md) how to save them.

## Grabbing a joint

Move the pointer over the figure and **press the left button on the part you want to move**. PoseStudio picks the skin under the pointer and the joint that moves that skin — the wrist when you click on a hand, the knee or the hip when you click along the thigh — and tints the flesh that joint drives blue, so you always see what you have hold of. The point under your pointer is what you are holding: drag, and that point follows the cursor.

- What you see is what you get: the click lands on the **surface in front**, so a part hidden behind another is never selected through it. Clicking the chest takes the chest, not the shoulder or the arm behind it; to grab the far arm, orbit until you can see it.
- Clicking a **joint** itself (a knee, an elbow, a wrist) takes hold of that joint.
- Clicking **along a limb** takes hold of the point you clicked: the far end of the limb segment when you click nearer it (the knee for a thigh, the hand for a forearm), the near end otherwise.
- A click on the **hips** takes hold of the pelvis; a click on the **chest** or the **head** takes hold of the trunk there.
- A click anywhere on the **face** takes hold of the **head**, and the whole head lights up. The face's own bones — the eyes, the jaw, the lips, the brows, the ears — are there for expressions and are not posed: nothing on the face is selected by itself, and no drag turns one.
- A click on a **finger** or a **toe** takes hold of that digit, at the point you clicked. See [Fingers and toes](#fingers-and-toes).
- Joints stay clickable whether the [skeleton overlay](shading.md#the-skeleton-overlay) is on or off. Turn it on to see exactly where they are. A click just outside the body still takes the nearest joint, as long as nothing is in front of it.

A left press that finds no joint orbits the camera instead, and a plain click selects or deselects the model.

## Dragging: full-body IK

**Left-drag a joint** and the body follows. The grabbed point tracks the cursor in the plane of the screen, and everything else is worked out for you, every frame:

- **The feet stay planted.** Standing feet are anchored to the floor — the ball of the foot holds hard, the heel may lift when the ankle runs out, and the toes stay flat. Nothing you drag slides a planted foot.
- **The body balances.** The weight stays over the feet: pull a hand forward and she leans; pull it further and the whole body leans as far as it can and then stops.
- **She steps when she must.** Take the hips, the chest or the head far enough across the floor and the feet step under the body, one at a time, each with a swing and a landing. A step in flight always finishes when you let go.
- **She crouches, kneels, sits and lies down.** Push the hips down and she crouches; take them down and forward and she kneels, the knees landing on the floor and the feet lying flat on their tops behind her, toes back (and the knees staying there when the chest is then pulled forward past her reach: she goes down onto her hands rather than up into a plank); down and back and she sits. Knees, hands, the seat and the head all become supports when they reach the floor and let go again when the body rises off them. See [Poses Step by Step](poses.md).
- **Joints stay within their limits.** Every joint has the anatomical range its figure was authored with — a knee bends one way, a neck turns so far — and a drag never takes a joint past it.
- **Limbs stay out of the body and out of each other.** Arms cross the chest without passing through it, a hand laid on the other forearm rests on it, crossed legs stay apart.
- **Reaching high, and lifting her off the floor.** Pull a hand up past the arm's reach and she rises onto tiptoe — heels up, body tall, the shoulder shrugging no further than a shoulder does; bring it back within reach and the heels come down. Pull steadily further up and, after a moment, she lifts off and hangs from it; lower her and she lands and stands back up.
- **A hand behind the head** is made the way you would make it: drag the hand up beside the head, then roll the mouse wheel a notch at a time to push it back. Dragged there in one diagonal motion the arm may swing back first and come round late; the path up and then back gives the pose cleanly.
- **A hand laid on the body lies on its palm.** Bring a hand to the hip, the chest, the top of the head, the belly or a knee and hold it there: it turns to lie flat, palm to the body, the way a hand put on the floor does. Brought to the small of the back it rests on the back of the hand instead — the forearm cannot turn the palm round that far. The hand settles only where it comes to rest; dragged past the body it keeps its angle. A hand that leaves the surface keeps the turn it took there; Reset Joint or an `X`/`Y`/`Z` wheel turn of the wrist straightens it.

The response is deliberately smooth: the figure follows the cursor with a small lag rather than snapping, and a still cursor gives a perfectly still figure. Releasing the button ends the drag; if a step or a landing was still in flight, it completes on its own, and then the drag becomes one entry in the undo history.

### Moving toward and away from the camera

The cursor moves the grabbed point across the screen. To move it **in depth** — toward or away from you — **roll the mouse wheel while the button is held**: scrolling up pushes the point away from the camera, scrolling down pulls it toward you, in steps of about 2% of its distance from the camera. The point stays under the pointer as its depth changes. Orbit to a side view to check the depth you reached.

### Special joints

Some joints have their own way of being dragged, because that is what a person means by moving them:

- **A knee** dragged swings over its planted foot — out, in, forward, back — while the foot pivots on its heel. Pull it up past what the shin allows and the foot lets go, trails, and swings in under the knee; bring it back down and it returns to its spot.
- **Lifting a foot shifts the weight.** As a foot leaves the floor — dragged up itself, or let go of by a raised knee — the hips move over the foot she stands on, the standing leg angles in under her, and the pelvis drops a little on the free side with the chest kept level, so a knee raise, a kick or a foot put up on a step is a stance she could hold. Bring the foot back down and the hips come back. A foot that was already in the air when you took hold of something shifts nothing.
- **Idle hands relax.** The fingers of an arm that hangs curl gently into a relaxed hand as you pose, and a hand coming down onto the floor opens again. Fingers you have turned yourself are left as you set them; Reset Pose flattens them.
- **Figures that import in a T-pose** (arms straight out to the sides) keep their arms out while you pose the rest: arms held out are posed arms, and only arms hanging at her sides follow the body as it leans, bows and crouches. Drag each hand down to her side first (or turn the shoulders with the `X`/`Y`/`Z` wheel), and the arms hang and move as they do on figures that import with their arms down.
- **A hand** taken down folds the trunk at the hips as it goes — a bow over softened knees to bring a hand to the knee, a fold at the hips and the knees to reach the floor — the hips going back to keep her balance and the head looking down where the hand goes. Bring the hand back up and she stands up with it.
- **An elbow** dragged swivels about a hand that stays where it is (elbows out, elbows in, an arm tucked); a straight arm's elbow carries the arm.
- **A foot** on the floor dragged along it slides: the hips come between the feet and the sole stays flat. A foot lifted in the air is placed by its leg, the weight going over the other foot; set down from a tilt, its sole rolls flat as it comes down.
- **The hips** dragged sideways sway — the pelvis rolls and the chest stays level; dragged back they hinge so the head comes forward over the feet; dragged across the floor they walk.
- **The head** dragged takes the body with it through the neck — it leans, steps, bows and crouches. To *turn* the head, rotate it instead (below).

### Fingers and toes

**A finger or a toe dragged moves that one digit and nothing else.** Every joint of the digit bends toward the cursor, up to where the digit joins the hand or the foot; the hand, the foot and the whole body stay exactly where they are. It makes no difference whether `Ctrl` is held.

- Take hold of a **fingertip** to curl or straighten the finger; take hold nearer the knuckle to swing the finger from the knuckle.
- Pulled beyond its reach, the digit points after the cursor as far as its joints allow and rests there. The joint limits and the floor still hold.
- To move the **hand** or the **foot** itself, take hold of it, not of a digit: the back of the hand, the palm or the wrist; the instep, the heel or the ankle.
- To turn a single finger joint about one axis, select it and use the `X`/`Y`/`Z` wheel (below).
- Zoom in before posing fingers. From a view of the whole figure a finger is a few pixels wide, and it is easy to take hold of the hand instead.

## Posing one part: Ctrl + drag

Sometimes you want to move one part and leave the rest exactly as it is. **Ctrl + left-drag** a limb or the head and only the chain it belongs to follows the cursor:

- **A hand, elbow, foot or knee**: the limb moves, up to where it joins the body — the arm with its collar bone, the leg from the hip. The torso, the head and the other limbs stay where they were. A knee dragged this way swings over its planted foot, as it does in a plain drag.
- **The head or neck**: the neck bends toward the cursor; the chest and everything below stay.
- **A finger or a toe**: that digit alone, as in a plain drag ([Fingers and toes](#fingers-and-toes)).

Nothing else happens in this mode: no balance, no step, no lift-off, and the arms and head are not re-posed for you. The joint limits, the floor and the body still hold, so the chain stops where it would collide. Pulled beyond the chain's reach, the part points toward the cursor and rests there. Each Ctrl-drag is one undo step, and the wheel's depth control works during it as in a plain drag.

## Moving the whole figure: Ctrl + drag the body

**Ctrl + left-drag the body itself** — the hips, the belly, the chest — and the whole figure follows the cursor exactly as she is posed. Nothing is anchored to the floor: her feet, or a knee or a hand she has on the floor, come along. No joint turns, she does not balance or step. Use it to lift a posed figure, carry her across the scene, or set her down somewhere else.

- **Up and across**: she goes where the cursor goes; roll the wheel while dragging to move her toward or away from the camera.
- **Down**: the floor stops her. She comes down until the lowest part of her pose rests on it and no further.
- **Your pins hold.** A joint you pinned stays exactly where you pinned it, and its limb gives so that it can: pin a hand and move the body, and the arm bends or straightens to keep the hand in place. When the limb has no more to give, the pin stops the body there. A pin on the hips or the torso holds the whole figure where she is.
- A figure left in the air stays in the air. Press the [Ground button](#the-ground-button) to drop her onto the floor.
- The move is one undo step, like any drag.

To *pose* the torso — lean, bow, crouch, walk — drag it without `Ctrl`.

## Rotating a joint: FK

To turn one joint about one axis — twist a forearm, tilt the head, cock a wrist — **hold `X`, `Y` or `Z` and roll the mouse wheel** with a joint selected: each wheel notch turns the joint 5° about that one channel of its rotation. A badge under the viewport strip shows the axis. This is the way to reach a joint's *twist*, which a drag cannot address. The hold also works in the middle of a drag: the wheel turns the grabbed joint while the body keeps following the cursor.

Rotations stop at the joint's limits, and a rotation that would push a limb into the body stops at the body. Each key-hold is one undo step.

## Joint pins

A **pin** holds a joint exactly where it is through every later drag: a hand kept on a hip, a foot that must stay on its spot, hips held on a chair.

- Select a joint (click it) and press **`P`** to pin it; press `P` again to unpin. The context menu (right-click a joint) offers **Pin Joint**, **Unpin Joint** and **Unpin All Joints**.
- Pinned joints show an **orange marker**. While you drag, the feet the body is standing on show smaller **cyan** markers — those are the drag's own contacts, not pins.
- A pin holds both the joint's place and its orientation, to microns, under any drag of another joint. Dragging the pinned joint itself moves it.
- **Pinning the hips makes a seat.** PoseStudio knows nothing about chairs, so this is how you tell it that something holds her up: sit her down by the hips, pin them, and pose the rest — balance and stepping stand down, and the pelvis rocks under a chest lean like a seated person's.
- Pins are part of the pose: they are saved in [pose files](pose-files.md), and undoing a drag restores the pins of that moment.

## The Ground button

A pose that bends the knees lifts the feet off the floor, because the hips stayed where they were. The **Ground** button on the viewport strip drops the active figure so the lowest point of its current pose rests on the floor — animated as a real fall (a metre takes about half a second). A figure sunk below the floor is lifted out at once. Grounding is refused while you are dragging. It is a pose edit like any other: one undo step (`Ctrl+Z` puts her back where she was), saved with the pose in a [pose file](pose-files.md), and taken back by **Reset Pose** along with everything else.

**She lands like a person.** A figure that comes down on her feet absorbs the landing: her knees fold, her hips dip over her planted feet, and she comes back up — all in under half a second. The higher the drop, the deeper the dip: a few centimetres' drop barely bends the knees, a drop from a metre or more is a real knee bend. It is for the eye only. When it is over she stands in exactly the pose you dropped her in: the dip itself leaves no trace, and undoing the drop is the one undo step.

- There is no bounce when she lands on anything but her feet — her knees, her hands, her seat — or when her hips are [pinned](#joint-pins).
- Clicking, pressing a key or starting any edit while she is still falling or dipping finishes it at once.

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

**Edit → Undo** (`Ctrl+Z`) and **Redo** (`Ctrl+Y`) walk one history of everything you changed: each drag, each rotation, each pin toggle, each utility, each drop to the ground and each pose file loaded — and each lighting change on the Environment tab. An entry is recorded when a gesture ends, so a long drag is one step. Deleting an object clears the history.

## Several figures

You can import as many figures as you like. Posing addresses the **active figure** — the one whose joint you clicked last — and the utilities, the pins, the pose files and the Ground button all act on it. Click a joint on another figure to make that one active. While a model that is not a figure (an `.obj`) is the selection, the posing utilities do nothing.

## What the figure knows

Everything the body does comes from the figure you imported, not from PoseStudio's guesses:

- **Joint limits** are the ranges authored in the figure. Locked channels (a twist bone's bending axes) never move.
- **Correctives** — the sculpted shapes that fix a deep elbow or knee bend — blend live while you drag, so the mesh looks right at every frame.
- **Body proportions** set every threshold: a small character crouches, steps and kneels at her own scale.

## Tips

- Orbit to a **side view** (`3`) before pushing a figure down onto the floor or forward onto her hands — you can see the depth, and the drag plane is the one you want.
- To move a whole figure, drag the hips: they walk her across the floor. To move her without changing her pose, [Ctrl + drag the body](#moving-the-whole-figure-ctrl--drag-the-body).
- If a drag does not go where you expect, look at where the feet are: a planted foot is an anchor, and a pull the body cannot follow without stepping ends in a lean. Take the hips there instead, or lift a foot first.
- A figure left kneeling, sitting or lying starts the next drag from that pose — pull the hips or the chest up to get her up again (a kneeler's feet roll back onto their toes as she rises, and her knees come up with them).
- Reset Pose is always one step away when a pose gets away from you.
