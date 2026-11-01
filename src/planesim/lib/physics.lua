-- Flight model for a small aerobatic plane.
--
-- World axes: X east, Y north, Z up, metres; the pilot stands at the origin
-- facing north. Body axes: X nose, Y left, Z up. The attitude is a rotation
-- matrix R (body -> world): its columns are the nose, left and up axes in
-- the world, stored as h.r11 .. h.r33 (row, column). Body rates p, q, r turn
-- about X, Y, Z: p > 0 rolls right, q > 0 pitches the nose down, r > 0 yaws
-- the nose left. Forces are per unit mass (accelerations).
--
-- Aerodynamics, kept simple but with the things a pilot feels:
--   * lift grows with airspeed squared and angle of attack up to a stall at
--     about 15 degrees, then drops to a flat-plate value
--   * drag: parasitic, induced, and a flat-plate term at high angles, so a
--     stalled or hovering plane sinks slowly
--   * prop thrust falls off as the airspeed nears the prop's pitch speed
--   * the controls command roll, pitch and yaw rates (like a stabilised
--     receiver in rate mode) with authority that fades as the airspeed
--     drops; the prop wash keeps the elevator and rudder working at low
--     speed, so the plane can hover on the prop
--   * the plane weathervanes into the airflow (pitch and yaw stability),
--     which is what makes the nose drop in a stall
-- On the ground it is a taildragger: it rolls on its wheels along its
-- heading, steers with the rudder, raises its tail as it speeds up and lifts
-- off when the wing carries its weight. step() allocates nothing.

local sqrt, abs, min, max, atan = math.sqrt, math.abs, math.min, math.max, math.atan
local sin, cos = math.sin, math.cos

local physics = {}

local G = 9.81
local THRUST_MAX = 1.7 * G     -- static thrust: more than the weight (prop hang)
local PROP_SPEED = 42          -- m/s where the prop stops giving thrust
local LIFT_K = 0.97            -- lift accel per (m/s)^2 per unit CL
local STALL = 0.26             -- rad: stall angle (CL = angle below it)
local CD0, CDI, CDA = 0.011, 0.30, 0.30
local SIDE_K = 0.06            -- side force per (m/s)^2 of sideslip
local V_FULL = 10              -- m/s for full control authority
local WASH = 160               -- prop wash at full throttle, (m/s)^2
local PITCH_STAB = 3.0         -- 1/s: weathervane in pitch at full airspeed
local YAW_STAB = 3.0
local TRIM_ALPHA = 0.05        -- rad: the angle of attack the plane is trimmed for
local LEVEL_GAIN = 6           -- self-level: rad/s per rad of attitude error
local LEVEL_MAX_ROLL = 1.2     -- self-level: tan of the full-stick bank (~50 deg)
local LEVEL_MAX_PITCH = 0.7    -- self-level: tan of the full-stick pitch (~35 deg)
local THROTTLE_RESPONSE = 7    -- 1/s

-- Ground.
local ROLL_FRICTION = 0.05     -- times g
local STEER_RATE = 1.4         -- rad/s at full rudder, slow
local TAIL_UP_SPEED = 7        -- m/s where the tail is fully up
local LAND_MAX_SINK = 3.0
local LAND_MAX_SLIP = 4.0
local LAND_MIN_UPRIGHT = 0.85

physics.G = G

-- Contact points (body axes), from the mesh.
local WX, WY, WZ, TX, TZ = 0.1, 0.16, -0.22, -0.77, -0.01
local TIPX, TIPY, TIPZ = 0.05, 0.75, -0.01
local NX, NZ, PX, PZ, PR = 0.42, 0.01, 0.35, 0.01, 0.2
local FX, FZ, CX, CZ = -0.71, 0.26, -0.19, 0.16
local REST_PITCH = 0.24

function physics.setGeometry(g)
  WX, WY, WZ, TX, TZ = g.WHEEL_X, g.WHEEL_Y, g.WHEEL_Z, g.TAIL_X, g.TAIL_Z
  TIPX, TIPY, TIPZ = g.TIP_X, g.TIP_Y, g.TIP_Z
  NX, NZ, PX, PZ, PR = g.NOSE_X, g.NOSE_Z, g.PROP_X, g.PROP_Z, g.PROP_R
  FX, FZ, CX, CZ = g.FIN_X, g.FIN_Z, g.CANOPY_X, g.CANOPY_Z
  -- Tail-down pitch with both wheels and the tail on the ground.
  REST_PITCH = atan(TZ - WZ, WX - TX)
end

-- Lift coefficient for an angle of attack (rad, -pi..pi).
local function liftCoef(a)
  if a <= STALL and a >= -STALL then return a end
  return 0.2 * sin(2 * a)
end

-- Set the attitude from heading (0 north, + towards east) and pitch (nose
-- up +), wings level.
local function setAttitude(h, heading, pitch)
  local sh, ch = sin(heading), cos(heading)
  local sp, cp = sin(pitch), cos(pitch)
  -- nose, left, up
  h.r11, h.r21, h.r31 = sh * cp, ch * cp, sp
  h.r12, h.r22, h.r32 = -ch, sh, 0
  h.r13, h.r23, h.r33 = -sh * sp, -ch * sp, cp
end

-- CG height that puts the wheels on the ground at a given pitch.
local function groundHeight(pitch)
  return -(WZ * cos(pitch) + WX * sin(pitch))
end

-- On the ground at (x, y), pointing along heading, stopped.
function physics.reset(h, x, y, heading)
  h.heading = heading or 0
  h.groundPitch = REST_PITCH
  setAttitude(h, h.heading, REST_PITCH)
  h.px, h.py, h.pz = x or 0, y or 0, groundHeight(REST_PITCH)
  h.vx, h.vy, h.vz = 0, 0, 0
  h.p, h.q, h.r = 0, 0, 0
  h.thr = 0
  h.speed = 0             -- ground roll speed along the heading
  h.airspeed = 0
  h.landed = true
  h.crashed = false
  h.crashReason = nil
end

local function crash(h, reason)
  h.crashed = true
  h.crashReason = reason
  h.vx, h.vy, h.vz = 0, 0, 0
  h.p, h.q, h.r = 0, 0, 0
  if h.pz < 0.15 then h.pz = 0.15 end
end

-- Thrust for a forward airspeed.
local function thrust(thr, u)
  local k = 1 - u / PROP_SPEED
  if k < 0 then k = 0 elseif k > 1.3 then k = 1.3 end
  return thr * THRUST_MAX * k
end

local function groundStep(h, inp, dt)
  local s = h.speed
  local thr = h.thr
  -- Tail comes up with speed; up elevator holds it down.
  local tailUp = s / TAIL_UP_SPEED
  if tailUp > 1 then tailUp = 1 end
  local pitch = REST_PITCH * (1 - tailUp)
  if inp.pitch < 0 then
    pitch = pitch - inp.pitch * REST_PITCH * tailUp
  end
  if pitch > REST_PITCH then pitch = REST_PITCH end
  h.groundPitch = pitch
  local cl = liftCoef(pitch + TRIM_ALPHA)
  local lift = LIFT_K * s * s * cl
  local drag = s * s * (CD0 + CDI * cl * cl)
  local friction = ROLL_FRICTION * max(0, G - lift)
  local a = thrust(thr, s) - drag
  if s > 0 or a > friction then
    a = a - friction
  else
    a = 0
  end
  s = s + a * dt
  if s < 0 then s = 0 end
  h.speed = s

  -- Steering: tail wheel when slow, rudder when fast.
  local steer = inp.yaw * STEER_RATE * min(1, s / 2 + 0.3)
  h.heading = h.heading + steer * dt

  setAttitude(h, h.heading, pitch)
  local sh, ch = sin(h.heading), cos(h.heading)
  h.vx, h.vy, h.vz = sh * s, ch * s, 0
  h.px, h.py = h.px + h.vx * dt, h.py + h.vy * dt
  h.pz = groundHeight(pitch)
  h.airspeed = s

  -- Lift off when the wing carries the weight.
  if lift > G * 1.02 then
    h.landed = false
    h.vz = 0.2
    h.p, h.q, h.r = 0, 0, 0
  end
end

-- inp: roll, pitch, yaw in -1..1 (shaped), thr 0..1, rollRate, pitchRate,
-- yawRate (rad/s at full stick), response (1/s), level (bool).
function physics.step(h, inp, dt)
  if h.crashed then return end

  h.thr = h.thr + (inp.thr - h.thr) * min(1, dt * THROTTLE_RESPONSE)
  local thr = h.thr

  if h.landed then
    groundStep(h, inp, dt)
    return
  end

  local r11, r12, r13 = h.r11, h.r12, h.r13
  local r21, r22, r23 = h.r21, h.r22, h.r23
  local r31, r32, r33 = h.r31, h.r32, h.r33
  local vx, vy, vz = h.vx, h.vy, h.vz

  -- Airflow in body axes.
  local u = r11 * vx + r21 * vy + r31 * vz
  local v = r12 * vx + r22 * vy + r32 * vz
  local w = r13 * vx + r23 * vy + r33 * vz
  local V2 = u * u + v * v + w * w
  local V = sqrt(V2)
  h.airspeed = V
  local alpha = atan(-w, u)

  -- Control authority: airspeed, plus prop wash on the tail.
  local effAir = V2 / (V_FULL * V_FULL)
  if effAir > 1 then effAir = 1 end
  local wash = WASH * thr
  local effTail = (V2 + wash) / (V_FULL * V_FULL)
  if effTail > 1 then effTail = 1 end
  local effAil = (V2 + 0.4 * wash) / (V_FULL * V_FULL)
  if effAil > 1 then effAil = 1 end

  -- Rate commands.
  local pc, qc
  if inp.level then
    -- Target up vector from the sticks, in the heading frame.
    local fx, fy = r11, r21
    local fn = sqrt(fx * fx + fy * fy)
    if fn < 1e-3 then fx, fy, fn = r13, r23, sqrt(r13 * r13 + r23 * r23) + 1e-6 end
    fx, fy = fx / fn, fy / fn
    local rx, ry = fy, -fx
    local tp, tr = inp.pitch * LEVEL_MAX_PITCH, inp.roll * LEVEL_MAX_ROLL
    local ux, uy, uz = fx * tp + rx * tr, fy * tp + ry * tr, 1
    local un = sqrt(ux * ux + uy * uy + uz * uz)
    ux, uy, uz = ux / un, uy / un, uz / un
    local ex = r23 * uz - r33 * uy
    local ey = r33 * ux - r13 * uz
    local ez = r13 * uy - r23 * ux
    if r33 < -0.95 and abs(ex) + abs(ey) + abs(ez) < 0.3 then ex, ey, ez = r11, r21, r31 end
    pc = (r11 * ex + r21 * ey + r31 * ez) * LEVEL_GAIN
    qc = (r12 * ex + r22 * ey + r32 * ez) * LEVEL_GAIN
    if pc > inp.rollRate then pc = inp.rollRate elseif pc < -inp.rollRate then pc = -inp.rollRate end
    if qc > inp.pitchRate then qc = inp.pitchRate elseif qc < -inp.pitchRate then qc = -inp.pitchRate end
    pc, qc = pc * effAil, qc * effTail
  else
    pc = inp.roll * inp.rollRate * effAil
    qc = inp.pitch * inp.pitchRate * effTail
  end
  local rc = -inp.yaw * inp.yawRate * effTail

  -- Weathervane: the nose follows the airflow.
  if V > 0.5 then
    qc = qc + PITCH_STAB * effAir * (-w / V - TRIM_ALPHA)
    rc = rc + YAW_STAB * effAir * (v / V)
  end

  local k = min(1, dt * inp.response)
  local p = h.p + (pc - h.p) * k
  local q = h.q + (qc - h.q) * k
  local r = h.r + (rc - h.r) * k
  h.p, h.q, h.r = p, q, r

  -- Attitude: R = R * (I + [w]x dt), re-orthonormalised.
  local wx, wy, wz = p * dt, q * dt, r * dt
  r11, r12, r13 = r11 + r12 * wz - r13 * wy, r12 - r11 * wz + r13 * wx, r13 + r11 * wy - r12 * wx
  r21, r22, r23 = r21 + r22 * wz - r23 * wy, r22 - r21 * wz + r23 * wx, r23 + r21 * wy - r22 * wx
  r31, r32, r33 = r31 + r32 * wz - r33 * wy, r32 - r31 * wz + r33 * wx, r33 + r31 * wy - r32 * wx
  local n = 1 / sqrt(r11 * r11 + r21 * r21 + r31 * r31)
  r11, r21, r31 = r11 * n, r21 * n, r31 * n
  local d = r11 * r12 + r21 * r22 + r31 * r32
  r12, r22, r32 = r12 - d * r11, r22 - d * r21, r32 - d * r31
  n = 1 / sqrt(r12 * r12 + r22 * r22 + r32 * r32)
  r12, r22, r32 = r12 * n, r22 * n, r32 * n
  r13 = r21 * r32 - r31 * r22
  r23 = r31 * r12 - r11 * r32
  r33 = r11 * r22 - r21 * r12
  h.r11, h.r12, h.r13 = r11, r12, r13
  h.r21, h.r22, h.r23 = r21, r22, r23
  h.r31, h.r32, h.r33 = r31, r32, r33

  -- Forces in body axes.
  local cl = liftCoef(alpha)
  local sa = alpha < 0 and -alpha or alpha
  if sa > 1.5708 then sa = 3.1416 - sa end
  local sinA = sin(sa)
  local vxz = sqrt(u * u + w * w)
  local fx, fy, fz = 0, 0, 0
  if vxz > 0.01 then
    -- Lift, perpendicular to the airflow in the plane of symmetry.
    local l = LIFT_K * vxz * cl
    fx, fz = -w * l, u * l
  end
  local dragK = CD0 + CDI * cl * cl + CDA * sinA * sinA
  fx = fx - dragK * V * u + thrust(thr, u)
  fy = -dragK * V * v - SIDE_K * V * v
  fz = fz - dragK * V * w

  local ax = r11 * fx + r12 * fy + r13 * fz
  local ay = r21 * fx + r22 * fy + r23 * fz
  local az = r31 * fx + r32 * fy + r33 * fz - G
  vx, vy, vz = vx + ax * dt, vy + ay * dt, vz + az * dt
  h.vx, h.vy, h.vz = vx, vy, vz
  local px, py, pz = h.px + vx * dt, h.py + vy * dt, h.pz + vz * dt
  h.px, h.py, h.pz = px, py, pz

  -- Ground contact; only worth checking near the ground.
  if pz > 2 then return end

  -- Wing tips (whichever is lower), prop disc, nose, fin, canopy.
  local tipZ = pz + r31 * TIPX + r33 * TIPZ - abs(r32 * TIPY)
  if tipZ < 0 then crash(h, "Wing strike") return end
  local hubZ = pz + r31 * PX + r33 * PZ
  if hubZ - PR * sqrt(r32 * r32 + r33 * r33) < 0 then crash(h, "Prop strike") return end
  if pz + r31 * NX + r33 * NZ < 0 then crash(h, "Nose in") return end
  if pz + r31 * FX + r33 * FZ < 0 or pz + r31 * CX + r33 * CZ < 0 then
    crash(h, "Flipped over") return
  end

  -- Wheels and tail.
  local wheelZ = pz + r31 * WX + r33 * WZ - abs(r32 * WY)
  local tailZ = pz + r31 * TX + r33 * TZ
  if wheelZ < 0 or tailZ < 0 then
    -- Speed along and across the heading.
    local hx, hy = r11, r21
    local hn = sqrt(hx * hx + hy * hy)
    if hn < 1e-3 then crash(h, "Crashed") return end
    hx, hy = hx / hn, hy / hn
    local along = vx * hx + vy * hy
    local across = vx * hy - vy * hx
    if r33 < LAND_MIN_UPRIGHT then
      crash(h, "Cartwheeled")
    elseif vz < -LAND_MAX_SINK then
      crash(h, "Hard landing")
    elseif abs(across) > LAND_MAX_SLIP or along < -1 then
      crash(h, "Ground loop")
    else
      h.landed = true
      h.speed = max(0, along)
      h.heading = atan(hx, hy)
      h.p, h.q, h.r = 0, 0, 0
      groundStep(h, inp, 0)
    end
  end
end

return physics
