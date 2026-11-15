# RobogamiRobotModule notes

## Default stance

The baseline `robogami` uses a fixed leg angle (`defaultLegAngleDeg`). The system-identified
variants (`robogami_small`, `robogami_medium`, `robogami_big`) compute their stance from the
height of the top hexagon above the bottom one (`stanceTopHeight`), keeping it level and centered.

Each leg is two halves of length `L` joined at the spherical joint, spanning from its motor pivot
(`l1` origin) to its top hinge (`l1topBase`). The leg is clamped onto its motor-driven plate at
offset `o` (`leg1_joint`) from the motor axis, and hinged on a plate under the top hexagon
(`leg1TopPlate_joint`, offset `tp`: once the top hexagon is level, that frame's Z points down, so
the hexagon's mid-plane is `-tp.z` above the hinge).

With the top hexagon level and centered above the bottom one, the top hinge sits `d` further out
(top hexagon radius - motor pivot radius) and `H` higher than the motor pivot. In the leg's plane,
seen from the motor pivot, the first half plus the plate offset is a rigid arm of length
`R = |(L - o.x, o.z)|` turned by `l1`, and the second half (length `L`) must close the gap to the
top pivot at distance `D = |(d, H)|`, which gives the triangle `R, L, D`:

```
l1 = m - g - a,  with  m = atan2(H, d),  a = atan2(o.z, L - o.x),
                       cos(g) = (D^2 + R^2 - L^2) / (2 D R)
second half's angle  b = atan2(H - (L - o.x) sin(l1) - o.z cos(l1),
                               d - (L - o.x) cos(l1) + o.z sin(l1))
x_l1roty = b - l1
l1topBase = pi - b      (keeps the top hexagon level)
l2TopMove = l3TopMove = 0
```

Legs 2 and 3 use the same angles by symmetry.

## Max distance limits

Each leg folds at its tip. For every leg, the lower-half and upper-half corners on the same side
(`distanceLimitCorners`) get two distance limits:

- a minimum (`iDistMin`, `sDistMin`): self-collision avoidance;
- a maximum: the security distance is `sDistMax = 2 * (leg_length - leg_side_length) *
  maxDistTipFraction` (twice the leg tip length, read from the URDF as the distance along the leg
  between the corner and the spherical joint at the tip), and the interaction distance
  `iDistMax = sDistMax - maxDistInteractionMargin`, where the QP starts slowing the tip opening.

`maxDistTipFraction` (default 0.9) and `maxDistInteractionMargin` (default 5 mm) are set in
`robogami.h`. With the current legs this gives 31.9/36.9 mm for the small/medium/big variants
(20.5 mm tip) and 32.0/37.0 mm for the baseline (20.6 mm tip).

## Top plate / motor box self-collision

For the detailed models (`robogami_small`, `robogami_medium`, `robogami_big`) one more essential distance limit keeps
the top hexagon plate (`top`) clear of the motor boxes: a limit between the top plate and the base plate (`base`),
offset by the height of the motor boxes standing on the base plate. Its security distance is the box height plus
`sDistTopMotor` (3 mm) and its interaction distance the box height plus `iDistTopMotor` (5 mm):

| Model | Motor box height | Security distance | Interaction distance |
|---|---|---|---|
| `robogami_small` | 20.0 mm | 23.0 mm | 25.0 mm |
| `robogami_medium`, `robogami_big` | 28.5 mm | 31.5 mm | 33.5 mm |

The box height is read from the URDF: the motor links' inertia is that of a uniform box, so
`I_xx = m (y^2 + z^2) / 12` and so on give its three sizes, and the motor joint's rotation gives which one is
vertical. The boxes sit directly on the base plate, so their height is the offset. The limit uses the existing
`top` and `base` convex hulls; no motor hulls are needed. The baseline `robogami` model has no motor boxes and
doesn't get this limit.

This is slightly more conservative than checking each box: it also keeps the top plate at that height where it is
over the base plate but not over a motor. In a 5x5x5 sweep of the motor angles over their range with `robogami_big`
it reached the same 60 of 125 targets, in the same poses, as without it: the leg limits keep the plate at least
34.7 mm above the base plate, so this limit never even reaches its interaction distance.
