# Experimental Rigify Feature Set

This provides a set of experimental Rigify rig types, some of which may be
included in Rigify in the future.

Use `Code > Download ZIP` to obtain a ZIP archive of the code, and install it as
a Feature Set through the Rigify Add-On settings.

## Limb Rigs

### Spline IK Tentacle (`limbs.spline_tentacle`)

This rig type implements a tentacle with an IK system using the Spline IK constraint.
The controls define control points of a Bezier curve, and the bone chain follows the curve.

The curve control points are sorted into three groups: start, middle and end. The middle
controls are always visible and active, while the other two types can be shown and hidden
dynamically using properties; when enabled they appear next to the corresponding permanent
start/end control and can be moved from there.

* **Extra Start Controls** specifies the number of optional start controls to generate.
* **Middle Controls** specifies the number of middle controls to generate.
* **Extra End Controls** specifies the number of optional end controls to generate.
* Curve Fit Mode:
  + **Stretch To Fit** stretches the whole bone chain to fit the length of the curve defined
    by the controls.
  + **Direct Tip Control** turns the last bone of the chain into the end control, allowing
    direct control over that bone, while the middle bones stretch to follow the curve and
    cover the gap. This is similar to how regular IK works for limbs.
  + **Manual Squash & Stretch** allows full manual control over the chain scaling, while the
    chain covers as much of the curve as it can given its current length.
* **Radius Scaling** allows scaling the controls to control the thickness of the chain through the curve.
* **Maximum Radius** specifies the maximum scale allowed by the *Radius Scaling* feature.
* **FK Controls** generates an FK control chain and IK-FK snapping.

**Runtime Options:**

* **Start Controls** changes the number of visible optional start controls.
* **End Controls** changes the number of visible optional end controls.
* **End Twist Fix** (Direct Tip Control only)
  For technical reasons, the rig can only determine the chain twist from the tip control
  within the -180..180 degrees range. Exceeding that flips the twist direction.
  This option allows working around the limitation by dialing in a rough estimate of
  twist in full rotations, and letting the rig auto-correct to the precise value within
  the 180 degrees range from the estimate.

## Spine Rigs

### BlenRig-like Spine (`spines.blenrig_spine`)

This implements an IK spine rig with controls behaving similar to BlenRig.

* **Custom Pivot Control** generates a movable pivot control for the torso.
* **Custom Hips Pivot** generates a movable pivot for the hip control.

**Runtime Options:**

* **FK Hips** allows the main hip control to fully control rotation of the hip bone.
* **FK Chest** releases the FK controls of the top of the spine from the IK mechanism.

## Body IK Rigs

In some rare cases, like crawling, it may be desirable to have IK controls that lock
the location of elbows/knees by adjusting the spine and shoulders. This group of
rigs contains extended versions of spine, shoulder, arm and leg rigs that provide
this functionality. Legs must be used in pair with a spine, and arms with shoulders.

### Spines

The feature set provides `body_ik.basic_spine` and `body_ik.blenrig_spine`, which
are extended versions of the standard spine and the BlenRig-like spine from this
feature set. They behave the same as the originals, except that they work with
the Body IK leg rig.

**Runtime Options:**

* **Snap To Hip IK** applies the adjustment from the Knee IK to the controls.

**Runtime Options (`body_ik.blenrig_spine`):**

Due to the way BlenRig spine works, it is possible to apply the effect of IK by
either offsetting the whole spine, or just the hip control.

* **Body IK Hips** switches to offsetting just the hip control.
* **Snap Hips To Hip IK** applies the hip control adjustment.

### Shoulder

The `body_ik.shoulder` rig implements a simple IK-compatible shoulder.

### Limbs

The `body_ik.arm` and `body_ik.leg` rigs extend the standard limbs to implement
the elbow/knee IK functionality. The rigs provide a second set of IK controls
mapped to the elbow/knee, and options for switching and IK-FK snapping.

The special IK is intended for poses that are very different from the default
rest pose, so it doesn't work that well if switched on immediately from rest.
For best result, the character should be pre-posed into a kneeling/crawling
pose using FK, and then switched to the IK controls using snapping. Knee IK
is also not stable for mathematical reasons when both legs are enabled and
the shins are parallel (basically there are infinitely many solutions and
it becomes confused).

**Runtime Options:**

* **IK Force Straight** enables the mechanism in the spine/shoulder to keep
  the limb fully extended with ordinary IK. This is obviously mutually exclusive
  with using the actual knee/elbow IK.

## Jiggle Rigs

These are rigs to provide jiggle behavior.

### Basic Jiggle (`jiggle.basic`)

Creates two grab controls with the deform bone automatically stretching between them.

The chain should consist of one or two bones. If present, constraints on
the ORG bones are transplanted to helper parent bones for the controls.

* **Master Control** generates a parent master control for the jiggle setup.
* **Follow Front** adds a constraint to apply part of the motion of the front control to the back.

### Cloth Jiggle (`jiggle.cloth_cage`)

A version of basic jiggle with support for a cloth simulation cage
that is used to deform a part of the final mesh via Surface Deform.

The intended setup is that the jiggle rig is used to deform the cage,
which permanently controls part of the final mesh, and has a simulation
that can be enabled and adjusted using the custom properties. To allow
attaching additional directly animated objects to the affected area, the
rig supports a feedback mechanism from the cage to the front control.

Custom properties on the cage object and mesh that have names starting
with 'option_' are automatically copied to the rig bone and linked
with drivers. Anchor empties parented to the cage are used to feed
the result of cloth simulation and/or cage shape keys to the rig.

Resetting all custom properties on the cage object and mesh to defaults,
and disabling the Armature modifier must always reconfigure it and the
anchors into the rest shape that the rig should bind to.

The cage can only depend on the first deform bone of this rig, while
the second deform is driven by cage feedback and should be used to
help transition between the cage affected area and pure bone rigging
on the main mesh.

* **Cloth Cage Mesh** (_required_) specifies the cage mesh object.
* **Front Anchor** specifies the empty parented to the cage and used for feeding
  motion of its front area back to the rig.
* **Shape Anchor** specifies an optional empty used to adjust the rig to the
  effect of shape keys pre-configuring the shape of the cage, using a linked
  duplicate based setup.
* **Only Use Shape Anchor Location** tells the rig to only use the translation
  of the shape anchor object for a simpler mechanism.

## Skin Rigs (Experimental)

These rigs implement a flexible system for rigging skin using multiple interacting
B-Bone chains. This is developed as a replacement for the Rigify face rig.

These rigs currently require a [custom build](https://builder.blender.org/download/temp-angavrilov-constraints/) of Blender due to pending patches.

The core idea of the system is that deformation is implemented using a standard
powerful B-Bone chain rig, while domain-specific rigs for the most part merely
add automation adjusting positions of the controls of their child chains. There
are also helper rigs that are useful for implementing simple custom effects directly
in the metarig.

The deformation part of the system consists of chains of one or more B-Bones connecting
control points (nodes). Whenever controls for two chains would completely overlap,
they are automatically merged.

For each merged control, one of the chains is selected as the owner, based on heuristic
factors like parent depth from root, presense of `.T`/`.B` `.L`/`.R` symmetry markers,
and even alphabetical order as the last resort. This can be overridden by an explicit
priority setting in cases when it guesses wrong.

The owner and its parents determine additional automation that is placed on the control.
As a special case, if a control is merged with its `.T`/`.B` `.L`/`.R` symmetry
counterparts (detected purely by naming), the automation from all of the symmetry
siblings of the owner is averaged.

### Basic Chain (`skin.basic_chain`)

This is the basic chain rig, which bridges controls with B-Bones but does not add
any automation to the controls themselves.

When controls are merely moved with Grab, the chains behave as if using standard
automatic handles, but rotating and optionally scaling the controls will adjust the result.

* **B-Bone Segments** specifies the number of segments to use. Setting this to 1 disables
  all advanced behavior and merely bridges the points with a Stretch To bone.
* **Merge Parent Rotation And Scale** can be enabled to let the chain respond to rotation
  and scale induced by parents of controls owned by other rigs that this chain merged into.
* **Use Handle Scale** enables using control scale to drive scale and/or easing of the B-Bone.
* **Connect With Mirror** specifies whether the ends of the chain should smoothly connect
  when merging controls with its `.T`/`.B` `.L`/`.R` symmetry counterpart. The relevant option
  must be enabled on both chains to work.
* **Connect Matching Ends** specifies whether the end of the chain should connect to
  the opposite end of a different chain when merging controls, forming a continuous smooth
  chain going in the same direction.  The relevant options must be enabled on both chains.
* **Sharpen Corner** specifies whether the rig should generate a mechanism to form a sharp
  corner at the relevant connected end, depending on the angle formed by control locations.
  When the control angle becomes sharper than the specified value, ease starts reducing from 1 to 0.
* **Orientation** specifies that the controls should be oriented the same as the specified
  bone, rather than being aligned to the chain. The option is accompanied with a custom
  Copy To Selected button that will only copy to selected rigs that have the same option,
  thus allowing indiscriminately selecting bones without assigning unnecessary values.
* **Chain Priority** allows overriding the control owner selection heuristic.

### Stretchy Chain (`skin.stretchy_chain`)

This rig extends the basic chain with automation that propagates movement of the start & end,
and an optional middle control, to other controls. This results in stretching the whole
chain when moving one of the ends, rather than just the immediatly adjacent B-Bones.

* **Middle Control Position** specifies the position of the middle control; disabled when 0.
* **Falloff** specifies the influence falloff curves of the start, middle and end controls.
  Zero results in linear falloff, increasing widens the influence, and -10 disables the
  influence propagation from that control completely.
* **Spherical Falloff** toggle buttons change the shape of the falloff curve from a power
  curve forming a parabola at falloff 1 (`1 - x^(2^f)`) to a curve forming a circle
  at falloff 1 (`(1 - x^(2^f))^(2^-f)`).
* **Falloff Along Chain Curve** computes the falloff curve along the length of the chain,
  instead of projecting on the straight line connecting its start and end points.
* **Propagate Twist** specifies whether twist of the chain should be propagated to control
  points between main controls.
* **Propagate Scale** specifies whether perpendicular scaling of the chain should be propagated
  to control points between main controls.
* **Propagate To Controls** allows other chains to see propagated twist and scale via
  *Merge Parent Rotation And Scale*, instead of it being completely local to the chain.
* **Primary Control Layers** optionally specifies layers for the end controls.
* **Secondary Control Layers** optionally specifies layers for the middle control, falling
  back to *Primary Control Layers* if not set.

The main controls with active falloff have the effect of *Merge Parent Rotation And Scale*
automatically enabled just for them.

### Anchor (`skin.anchor`)

This rig effectively acts as a zero-length chain with highest priority, ensuring
that it becomes the owner when merging controls with other chains, and allowing
one to input custom automation influence into the skin system.

All constraints on the metarig bone are moved to the created control.

* **Generate Deform Bone** creates a deformation bone parented to the control.
* **Suppress Control** makes the control a hidden mechanism bone.
* **Widget Type** selects which widget to generate.
* **Relink Constraints** operates the same as e.g. `basic.super_copy`.
* **Orientation** specifies the bone used to orient the control.

### Glue (`skin.glue`)

This rig is somewhat similar to Anchor, but instead of overriding controls,
it is used to read or adjust the state of controls generated by other rigs.
The head of the bone must overlap a control of another skin rig.

The rig sets up its ORG bone to read the state of the control, while
moving all constraints that were originally on the bone to the control.

* Glue Mode:
  + **Child Of Control** makes the ORG bone a child of the control bone.
  + **Mirror Of Control** makes the ORG bone a sibling of the control with
    a Copy Transforms constraint from the control. The resulting Local Space
    transformation is the same as control's Local Space.
  + **Mirror With Parents** parents the ORG bone to the parent automation
    a control owned by the glue rig would have had, while making it
    follow the actual control. This includes both direct and parent-induced
    motion of the control into the local space of the bone.
* **Relink Constraints** operates the same as e.g. `basic.super_copy`.
* **Use Tail Target** relinks `TARGET` or any constraints with empty target
  bone and no relink specification to reference the control located at the
  tail of the glue bone.
* **Target Local With Parents** switches tail target to operate similarly
  to *Mirror With Parents*.
* **Add Constraint** allows easily adding some typical glue constraints with
  specific **Influence**, as if they were at the start of the ORG constraint stack

### Basic Transform (`skin.transform.basic`)

This rig provides very simple parent automation, which uses regular
translation/rotation/scale to modify locations but not orientations
or scale of its child chain controls.

* **Generate Control** specifies whether to generate a visible control,
  or use the transformation of the ORG bone as a part of more complex
  ad-hoc rig setup.

## Face Rigs (Experimental)

These rigs extend the generic skin system with some mechanisms specific for the face.

### Eye Rig (`face.skin_eye`)

The rig must have two child skin chains, with names tagged with `.T` and `.B` symmetry
to mark the top and bottom eyelid. The chains are rigged to follow the surface of the
eye and twist to its normal. In addition, it creates controls to aim the eye, including
a master control shared by all eyes with the same parent, and the eyelids are
rigged to follow the movement of the eyeball with adjustable influence.

* **Eyeball And Iris Deforms** generates deform bones for the eyeball and the iris,
  the latter copying XZ scale from the eye target. The iris is located at the tail
  of the ORG bone.
* **Eyelid Detach Option** generates a slider to disable the mechanism that keeps
  eyelid controls stuck to the surface of the eye.

### Jaw Rig (`face.skin_jaw`)

The rig must have one or more lip loops, each formed by 4 child skin chains tagged
with `.T`/`.B` `.L`/`.R` symmetrical names.

The lip loops are sorted into layers based on the distance from corners to the common
center and rigged with blended influence of the jaw and the master mouth control. Other
child rigs simply become children of the jaw.

* **Bottom Lip Influence** specifies the influence of the jaw on the inner bottom lip
  with mouth lock disabled.
* **Locked Influence** specifies the influence of the jaw on both lips of locked mouth.
* **Secondary Influence Falloff** specifies the factor by which influence blends away
  with each successive lip loop (for bottom lip loops the blend moves away from inner
  bottom lip to full jaw influence).
