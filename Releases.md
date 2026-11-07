# 1.2.0

- Higher frame rate on the radio (about 50-70% more fps, ported from HeliSim):
  - Runs full screen, without the title bar.
  - Only the parts of the screen that change are redrawn; the flight data sits on bars above and below the scene and is redrawn every other frame.
  - Faster scenery drawing: trees are a round crown on a trunk, and Low no longer draws the mown strip's edge lines.
  - The physics steps run in one call (same flight behaviour, bit for bit).
- The plane is drawn where it is at the moment of the frame, and the follow camera's lag no longer changes with the frame time, so fast passes no longer stutter or jump.
- The CPU stats show fps, wakeups per second and the mean wakeup and paint time.
- Thanks to b14ckyy for this work.

# 1.1.0

- New **Detail** setting, default **Low**, for a higher frame rate on the radio: flat hills and tree line, no mown strip, one-triangle trees, a two-triangle shadow and the low detail model until the plane is close. **High** is the previous look.

# 1.0.0

- Initial release of PlaneSim.
- Low-poly Extra-style aerobatic plane with moving ailerons, elevator and rudder, a prop that blurs into a disc under power, white undersides and a ground shadow.
- Fixed-wing flight model: lift with a stall, drag, prop thrust and prop wash, weathervane stability, taildragger take-off and landing.
- Rate and self-level flight modes; roll, pitch and yaw rates, expo and response in the settings.
- Flying field with a runway, hills, a tree line and trees; the camera follows the plane from the pilot's position.
