# Hardware Design - Stage 2

## Component Selection
<!-- table: component | qty | key spec | why chosen -->

| Component | Qty | Key spec | Why chosen |
|---|---|---|---|
| | | | |
| | | | |
| | | | |
| | | | |
| | | | |
| | | | |
| | | | |
| | | | |

## Mechanical Design
### Chassis Layout
<!-- platform dimensions, spacing, what's on each level, why stacked -->

The chasis is a three-platform stack. The bottom platform has motor mounting brackets built into it, holding the motors beneath it, with the battery carried ontop. The middle platform holds the motor driver and buck converter. The top platform carries the perfboard with the Arduino nano and IMU, mounted centrally.

The battery was originally intended to be on the middle platform in order to avoid lowering center of mass, however a decision was made to mount it into a printed bracket for removability. The bracket takes up more space than the battery alone so space was only found for it in the bottom platform, this works against the stacks layout intent of raising it but was accepted in exchange for a removable battery.

### Printed Parts
<!-- bottom platform, motor rear supports, battery casing, perfboard clips -->

**Bottom Platform.** Printed as a single piece with the motor mounting brackets included in the geometry rather than as separate bonded parts. The brackets locate the motors at the front of the body only.

**Motor rear supports.** The integral brackets hold the motors at one end, leaving the rear of each motor unsupported. Because the motors are back-heavy, this caused the rear ends to slack under their own weight. A support was printed for each motor with a semicircular cutout matching the motor barrel radius, bonded to both the platform and the motor body. This constrains the rear of the motor and eliminates the issue.

**Battery bracket.** A printed enclosure glued to the bottom platform, holding the LiPo. The front face is left open so the battery slides in and out, and the top face is reduced to narrow lips along each side, retaining the battery vertically while leaving most of the top open to reduce excessive friction.

**Perfboard clips.** The top perfboard is populated with wiring on its underside, so it cannot sit flush against the platform. Friction-fit clips were printed to engage both the platform and the perfboard, standing it off far enough to clear the wiring beneath. Using a friction fit rather than glue means the perfboard can be lifted off for access without breaking the mounting. 2 clips were added one on each side near the edge of the platform.

### Assembly
The three platforms are joined by nylon M3 standoffs at [X]mm spacing.

The middle perfboard is mounted using four small M3 standoffs, glued to the platform and passing through the 4 corner holes with a screw ontop of each. The top perfboard uses friction-fit clips instead, which was the later and better approach since it avoids the bonded joint and allows the board to be lifted off. However, omponents on both boards are socketed on headers rather than soldered directly, so they remain accessible for replacement regardless of how the board itself is mounted.

Motors are fastened into the integral brackets with screws.

## Electronics
### Power Architecture
<!-- 2S LiPo, buck to 5V logic, separate rails, why -->
### Wiring
<!-- pin assignments, encoder channels, STBY -->

## Physical Parameters
<!-- mass, CoM height, wheel radius, encoder CPR, deadband - measured at home -->