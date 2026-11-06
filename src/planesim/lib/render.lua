-- The view: the pilot stands at the origin of the flying field and turns
-- their head to follow the plane, as when flying a real one.
--
-- render.frame() runs in wakeup and turns the scene into flat display lists
-- (sky and ground bands, triangles, lines, the plane's sorted faces);
-- render.paint() only replays them. Ethos aborts a paint that runs about
-- 20000 Lua instructions (measured in the Ethos simulator: "Max instructions
-- count reached"), while wakeup is not held to that budget, so all the maths
-- lives in wakeup (about 20000 instructions a frame) and paint stays near
-- 5000. No tables are allocated per frame.
--
-- Two detail levels (render.setDetail). Low, the default, is for the
-- radio: the hills and tree line are flat bands, there is no mown strip
-- (nor its edge lines), the shadow is two triangles, and the low detail
-- model is used unless the aircraft is close. High draws the silhouettes,
-- the strip and the rest.
-- Trees are the same at both: a rectangle trunk and a circle crown.
--
-- World axes: X east, Y north, Z up (metres). Camera yaw 0 looks north,
-- yaw > 0 turns towards east; pitch > 0 looks up. The camera never rolls,
-- so the horizon is always a level line.

local sin, cos, tan, atan = math.sin, math.cos, math.tan, math.atan
local sqrt, floor, min, max, abs = math.sqrt, math.floor, math.min, math.max, math.abs
local rad, pi = math.rad, math.pi
local t_sort = table.sort

local render = {}

local model                       -- the mesh module, from render.init()

local EYE_X, EYE_Y, EYE_Z = 0, 0, 1.7
local HFOV = rad(60)
local NEAR = 0.3
local PITCH_MIN, PITCH_MAX = rad(-50), rad(80)

---------------------------------------------------------------------------
-- Colours, built once.
---------------------------------------------------------------------------

local C = {}
local SHADES = {}
local SHADE_LEVELS = 8
local AMBIENT, DIFFUSE = 0.45, 0.65

local MODEL_COLORS = {
  {210, 40, 45},      -- body
  {78, 80, 88},       -- control surfaces
  {238, 238, 232},    -- white: spinner, gear, belly, undersides
  {95, 125, 160},     -- canopy
  {150, 150, 155},    -- prop blades
}

local SKY_RGB = {{192, 216, 236}, {156, 194, 230}, {118, 166, 222}, {84, 136, 208}}
local GROUND_RGB = {{94, 124, 80}, {82, 132, 60}, {76, 140, 56}}

-- Sky and ground as full-width bands by elevation (degrees), top down:
-- the edges and the colour of the band below each edge, one list per
-- detail level. At low detail the hills and the tree line are two of
-- the bands.
local BANDS = {}

local function rgb(t, a)
  return lcd.RGB(t[1], t[2], t[3], a)
end

local function buildColors()
  for m = 1, #MODEL_COLORS do
    local base = MODEL_COLORS[m]
    local levels = {}
    for i = 1, SHADE_LEVELS do
      local s = AMBIENT + DIFFUSE * (i - 1) / (SHADE_LEVELS - 1)
      levels[i] = lcd.RGB(min(255, floor(base[1] * s)), min(255, floor(base[2] * s)), min(255, floor(base[3] * s)))
    end
    SHADES[m] = levels
  end
  C.disc = lcd.RGB(30, 30, 34, 0.22)
  C.dot = lcd.RGB(210, 40, 45)
  C.sky = {}
  for i = 1, #SKY_RGB do C.sky[i] = rgb(SKY_RGB[i]) end
  C.ground = {}
  for i = 1, #GROUND_RGB do C.ground[i] = rgb(GROUND_RGB[i]) end
  C.hills = lcd.RGB(122, 150, 150)
  C.treeline = lcd.RGB(46, 84, 50)
  C.strip = lcd.RGB(104, 160, 70)
  C.runway = lcd.RGB(92, 94, 98)
  C.runwayMark = lcd.RGB(240, 240, 232)
  C.line = lcd.RGB(214, 230, 196)
  C.shadow = lcd.RGB(0, 0, 0, 0.35)
  C.trunk = lcd.RGB(92, 70, 50)
  C.crown = {lcd.RGB(34, 82, 44), lcd.RGB(44, 96, 50), lcd.RGB(28, 70, 40)}

  local function bandList(E, cols)
    local R = {}
    for i = 1, #E do R[i] = rad(E[i]) end
    return {R = R, C = cols}
  end
  local sky, ground = C.sky, C.ground
  BANDS[1] = bandList({92, 35, 14, 4, 3.4, 1.5, 0, -1.5, -8, -92},
    {sky[4], sky[3], sky[2], sky[1], C.hills, C.treeline, ground[1], ground[2], ground[3]})
  BANDS[2] = bandList({92, 35, 14, 4, 0, -1.5, -8, -92},
    {sky[4], sky[3], sky[2], sky[1], ground[1], ground[2], ground[3]})
end

---------------------------------------------------------------------------
-- The scenery.
---------------------------------------------------------------------------

-- Far hills and a nearer tree line, as rings of silhouette points:
-- elevation in degrees at evenly spaced azimuths (index 1 = north).
local HILLS = {3.0, 4.6, 3.2, 2.4, 3.8, 6.0, 4.2, 2.4, 2.6, 4.0, 4.6, 2.8, 2.4, 3.0, 2.6, 2.8}
local TREELINE = {}
do
  -- A fixed jagged pattern: deterministic so the scene never changes.
  local seed = 7
  for i = 1, 32 do
    seed = (seed * 1103515245 + 12345) % 2147483648
    TREELINE[i] = 1.0 + (seed % 1000) / 1000 * 1.3 + ((i % 3 == 0) and 0.5 or 0)
  end
end

local function ringTables(elev)
  local n = #elev
  local r = {n = n, step = 2 * pi / n, SA = {}, CA = {}, SE = {}, CE = {}}
  for i = 1, n do
    local a = (i - 1) * r.step
    r.SA[i], r.CA[i] = sin(a), cos(a)
    local e = rad(elev[i])
    r.SE[i], r.CE[i] = sin(e), cos(e)
  end
  return r
end

local hillRing, treeRing

local detail = 1                     -- 1 low (default), 2 high

-- The mown strip and the runway along it, east-west in front of the pilot
-- (world x, y). The plane starts at the west end, pointing east.
local STRIP_X0, STRIP_X1, STRIP_Y0, STRIP_Y1 = -80, 80, 4, 30
local RUNWAY_X0, RUNWAY_X1, RUNWAY_Y0, RUNWAY_Y1 = -60, 60, 13, 19
render.START_X, render.START_Y, render.START_HEADING = -30, 16, pi / 2

-- Trees: x, y, height, width, colour index; sorted far to near at init.
local TREES = {
  {-62, 88, 14, 6, 1}, {34, 98, 14, 6, 2}, {96, 14, 14, 6, 3}, {-98, 20, 15, 6.5, 1},
  {86, -34, 16, 7, 2}, {-26, -76, 13, 6, 3}, {-48, 34, 10, 5, 2}, {52, 30, 11, 5, 3},
}
local TREE_DIST = {}

---------------------------------------------------------------------------
-- Camera and display lists.
---------------------------------------------------------------------------

local camYaw, camPitch, zoom = 0, 0, 1
local cRx, cRy                       -- right (z = 0)
local cUx, cUy, cUz                  -- up
local cFx, cFy, cFz                  -- forward
local scrW, scrH, cx, cy, focal = 800, 480, 400, 240, 571
local scrTop = 0                     -- the scene's top row (below the HUD bar)
local kx, ky = 1, 1                  -- frustum slopes (with margin)

-- Bands: full-width rectangles.
local BY, BH, BCOL = {}, {}, {}
local bandCount = 0

-- Triangles: the ground layer.
local TX1, TY1, TX2, TY2, TX3, TY3, TCOL = {}, {}, {}, {}, {}, {}, {}
local triCount = 0

-- Trees, far to near: trunk rectangles and crown circles. The first
-- treeFar are behind the plane, the rest in front of it.
local KX, KY, KW, KH = {}, {}, {}, {}
local RX, RY, RR, RCOL = {}, {}, {}, {}
local treeCount, treeFar = 0, 0

-- Lines (field markings), drawn over the ground layer.
local LX1, LY1, LX2, LY2, LCOL = {}, {}, {}, {}, {}
local lineCount = 0

-- The plane: screen vertices, sorted face keys, face colours.
local SX, SY = {}, {}
local KEY, FCOL = {}, {}
local keyCount = 0
local modelMode = 0                   -- 0 hidden, 1 dot, 2 mesh
local dotX, dotY, dotR = 0, 0, 2
local KEY_FACES = 512

local function tri(x1, y1, x2, y2, x3, y3, col)
  local n = triCount + 1
  triCount = n
  TX1[n], TY1[n], TX2[n], TY2[n], TX3[n], TY3[n], TCOL[n] = x1, y1, x2, y2, x3, y3, col
end

-- Camera coordinates of a world point.
local function toCam(x, y, z)
  local dx, dy, dz = x - EYE_X, y - EYE_Y, z - EYE_Z
  return dx * cRx + dy * cRy, dx * cUx + dy * cUy + dz * cUz, dx * cFx + dy * cFy + dz * cFz
end

-- Clip a convex polygon (camera coordinates) to the view frustum, then fan
-- it into triangles. Work buffers are reused.
local PX, PY, PZ, QX, QY, QZ = {}, {}, {}, {}, {}, {}

local function clipPlane(n, a, b, c, d, sx, sy, sz, dx, dy, dz)
  -- Keep a*x + b*y + c*z + d >= 0.
  local m = 0
  if n == 0 then return 0 end
  local px, py, pz = sx[n], sy[n], sz[n]
  local pd = a * px + b * py + c * pz + d
  for i = 1, n do
    local qx, qy, qz = sx[i], sy[i], sz[i]
    local qd = a * qx + b * qy + c * qz + d
    if (pd >= 0) ~= (qd >= 0) then
      local t = pd / (pd - qd)
      m = m + 1
      dx[m], dy[m], dz[m] = px + (qx - px) * t, py + (qy - py) * t, pz + (qz - pz) * t
    end
    if qd >= 0 then
      m = m + 1
      dx[m], dy[m], dz[m] = qx, qy, qz
    end
    px, py, pz, pd = qx, qy, qz, qd
  end
  return m
end

-- Fan a projected convex polygon (camera coordinates in X, Y, Z) into the
-- triangle list. Written out rather than calling tri(): on the radio a Lua
-- call costs about as much as ten instructions.
local function fan(n, X, Y, Z, col)
  local f, ccx, ccy = focal, cx, cy
  local X1, Y1, X2, Y2, X3, Y3, CO = TX1, TY1, TX2, TY2, TX3, TY3, TCOL
  local t = triCount
  local x0, y0 = ccx + X[1] * f / Z[1], ccy - Y[1] * f / Z[1]
  local xp, yp = ccx + X[2] * f / Z[2], ccy - Y[2] * f / Z[2]
  for i = 3, n do
    local z = Z[i]
    local xi, yi = ccx + X[i] * f / z, ccy - Y[i] * f / z
    t = t + 1
    X1[t], Y1[t], X2[t], Y2[t], X3[t], Y3[t], CO[t] = x0, y0, xp, yp, xi, yi, col
    xp, yp = xi, yi
  end
  triCount = t
end

local function polygon(n, col)
  -- Outcodes: skip a polygon wholly outside one side of the view, project
  -- one wholly inside as it is, clip the rest.
  local any, all = 0, 31
  local kx_, ky_ = kx, ky
  local X, Y, Z = PX, PY, PZ
  for i = 1, n do
    local x, y, z = X[i], Y[i], Z[i]
    local code = 0
    if z < NEAR then code = 1 end
    local zx, zy = kx_ * z, ky_ * z
    if x > zx then code = code | 2 elseif -x > zx then code = code | 4 end
    if y > zy then code = code | 8 elseif -y > zy then code = code | 16 end
    any, all = any | code, all & code
  end
  if all ~= 0 then return end
  if any == 0 then
    fan(n, PX, PY, PZ, col)
    return
  end
  n = clipPlane(n, 0, 0, 1, -NEAR, PX, PY, PZ, QX, QY, QZ)
  n = clipPlane(n, -1, 0, kx, 0, QX, QY, QZ, PX, PY, PZ)
  n = clipPlane(n, 1, 0, kx, 0, PX, PY, PZ, QX, QY, QZ)
  n = clipPlane(n, 0, -1, ky, 0, QX, QY, QZ, PX, PY, PZ)
  n = clipPlane(n, 0, 1, ky, 0, PX, PY, PZ, QX, QY, QZ)
  if n < 3 then return end
  fan(n, QX, QY, QZ, col)
end

local function groundQuad(x0, y0, x1, y1, col)
  PX[1], PY[1], PZ[1] = toCam(x0, y0, 0)
  PX[2], PY[2], PZ[2] = toCam(x1, y0, 0)
  PX[3], PY[3], PZ[3] = toCam(x1, y1, 0)
  PX[4], PY[4], PZ[4] = toCam(x0, y1, 0)
  polygon(4, col)
end

-- Unit circles for the ground ellipses, by point count.
local CIRCLE = {}
for _, n in ipairs({4, 6, 8}) do
  local C, S = {}, {}
  for k = 1, n do
    local a = (k - 1) * 2 * pi / n
    C[k], S[k] = cos(a), sin(a)
  end
  CIRCLE[n] = {C = C, S = S}
end

-- A flat ellipse on the ground: centre, half axes along a unit direction.
-- The camera transform is written out (z = 0 for every point).
local function groundEllipse(gx, gy, ux, uy, ra, rb, n, col)
  local U = CIRCLE[n]
  local UC, US = U.C, U.S
  local rx, ry, upx, upy, fwx, fwy = cRx, cRy, cUx, cUy, cFx, cFy
  local ox, oy = gx - EYE_X, gy - EYE_Y
  local uz0, fz0 = -EYE_Z * cUz, -EYE_Z * cFz
  local X, Y, Z = PX, PY, PZ
  for k = 1, n do
    local ca, sa = UC[k] * ra, US[k] * rb
    local dx, dy = ox + ux * ca - uy * sa, oy + uy * ca + ux * sa
    X[k] = dx * rx + dy * ry
    Y[k] = dx * upx + dy * upy + uz0
    Z[k] = dx * fwx + dy * fwy + fz0
  end
  polygon(n, col)
end

-- Narrow the segment's parameter range [t0, t1] to where d >= 0 (d is the
-- signed distance at each end); nil when nothing is left.
local function clipT(da, db, t0, t1)
  if da < 0 and db < 0 then return nil end
  if da < 0 then t0 = max(t0, da / (da - db)) elseif db < 0 then t1 = min(t1, da / (da - db)) end
  if t0 >= t1 then return nil end
  return t0, t1
end

-- A world line, clipped to the near plane and the sides of the view.
local function line(x0, y0, z0, x1, y1, z1, col)
  local rx, ry, upx, upy, upz, fwx, fwy, fwz = cRx, cRy, cUx, cUy, cUz, cFx, cFy, cFz
  local dx, dy, dz = x0 - EYE_X, y0 - EYE_Y, z0 - EYE_Z
  local ax, ay, az = dx * rx + dy * ry, dx * upx + dy * upy + dz * upz, dx * fwx + dy * fwy + dz * fwz
  dx, dy, dz = x1 - EYE_X, y1 - EYE_Y, z1 - EYE_Z
  local bx, by, bz = dx * rx + dy * ry, dx * upx + dy * upy + dz * upz, dx * fwx + dy * fwy + dz * fwz
  -- Both ends inside the view: no clipping.
  if az >= NEAR and bz >= NEAR then
    local kxa, kxb, kya, kyb = kx * az, kx * bz, ky * az, ky * bz
    if ax <= kxa and -ax <= kxa and bx <= kxb and -bx <= kxb
      and ay <= kya and -ay <= kya and by <= kyb and -by <= kyb then
      local n = lineCount + 1
      lineCount = n
      local f = focal
      LX1[n], LY1[n] = cx + ax * f / az, cy - ay * f / az
      LX2[n], LY2[n] = cx + bx * f / bz, cy - by * f / bz
      LCOL[n] = col
      return
    end
  end
  local t0, t1 = clipT(az - NEAR, bz - NEAR, 0, 1)
  if t0 then t0, t1 = clipT(kx * az - ax, kx * bz - bx, t0, t1) end
  if t0 then t0, t1 = clipT(kx * az + ax, kx * bz + bx, t0, t1) end
  if t0 then t0, t1 = clipT(ky * az - ay, ky * bz - by, t0, t1) end
  if t0 then t0, t1 = clipT(ky * az + ay, ky * bz + by, t0, t1) end
  if not t0 then return end
  local dx, dy, dz = bx - ax, by - ay, bz - az
  local px, py, pz = ax + dx * t0, ay + dy * t0, az + dz * t0
  local qx, qy, qz = ax + dx * t1, ay + dy * t1, az + dz * t1
  local n = lineCount + 1
  lineCount = n
  LX1[n], LY1[n] = cx + px * focal / pz, cy - py * focal / pz
  LX2[n], LY2[n] = cx + qx * focal / qz, cy - qy * focal / qz
  LCOL[n] = col
end

-- A ring of silhouette points between the horizon and its elevations.
local function silhouette(r, col)
  local cp, sp = cos(camPitch), sin(camPitch)
  local sy, cyw = sin(camYaw), cos(camYaw)
  -- Only the points within the view (plus one either side).
  local half = atan(kx) + r.step
  local i0 = floor((camYaw - half) / r.step)
  local i1 = floor((camYaw + half) / r.step) + 1
  local SA, CA, SE, CE, n = r.SA, r.CA, r.SE, r.CE, r.n
  local pfx, pfy, ptx, pty, havePrev
  local f = focal
  for j = i0, i1 do
    local i = (j % n) + 1
    -- sin/cos of (azimuth - camera yaw)
    local sd = SA[i] * cyw - CA[i] * sy
    local cd = CA[i] * cyw + SA[i] * sy
    local zf = cd * cp                       -- foot: elevation 0
    local ce, se = CE[i], SE[i]
    local zt = ce * cd * cp + se * sp
    if zf > 0.2 and zt > 0.2 then
      local fx, fy = cx + sd * f / zf, cy + cd * sp * f / zf
      local tx, ty = cx + ce * sd * f / zt, cy - (se * cp - ce * cd * sp) * f / zt
      if havePrev then
        tri(pfx, pfy, ptx, pty, tx, ty, col)
        tri(pfx, pfy, tx, ty, fx, fy, col)
      end
      pfx, pfy, ptx, pty, havePrev = fx, fy, tx, ty, true
    else
      havePrev = false
    end
  end
end

-- A tree: a round crown (a sphere, which projects to a circle at any view)
-- on a rectangular trunk. The lcd fills circles about 2.5 times and
-- rectangles about 4 times as fast as triangles (rallybench P06/P07/P11).
-- The trunk does not lean with the perspective; it is thin and short.
local CROWN_R, TRUNK_W = 0.85, 0.12   -- crown radius and trunk width per tree width
local function tree(t)
  local x, y, h, w = t[1], t[2], t[3], t[4]
  -- The foot in camera coordinates; a point above it is that height along
  -- the camera's view of the world up axis (the right axis has no z part).
  local dx, dy = x - EYE_X, y - EYE_Y
  local bz = dx * cFx + dy * cFy - EYE_Z * cFz
  if bz < 2 then return end
  local bx = dx * cRx + dy * cRy
  local lim = kx * bz + w
  if bx > lim or -bx > lim then return end
  local r = w * CROWN_R
  local hc = h - r                      -- height of the crown centre
  local cz = bz + hc * cFz
  if cz < 2 then return end
  local by = dx * cUx + dy * cUy - EYE_Z * cUz
  local f = focal
  local sbx, sby = cx + bx * f / bz, cy - by * f / bz
  local scx, scy = cx + bx * f / cz, cy - (by + hc * cUz) * f / cz
  local n = treeCount + 1
  treeCount = n
  RX[n], RY[n], RR[n], RCOL[n] = scx // 1, scy // 1, (r * f / cz) // 1 + 1, C.crown[t[5]]
  -- Trunk from the foot up to the crown centre, centred between the two.
  local tw = (w * TRUNK_W * f / bz) // 1
  if tw < 1 then tw = 1 end
  local top = scy // 1
  KX[n], KY[n], KW[n], KH[n] = ((sbx + scx) * 0.5 - tw * 0.5) // 1, top, tw, sby // 1 - top
end

---------------------------------------------------------------------------
-- The plane.
---------------------------------------------------------------------------

-- Sun direction (towards the sun): high, behind the pilot's right shoulder,
-- so the side of the model facing the pilot is lit.
local SUN_X, SUN_Y, SUN_Z
do
  local x, y, z = 0.35, -0.55, 0.78
  local l = sqrt(x * x + y * y + z * z)
  SUN_X, SUN_Y, SUN_Z = x / l, y / l, z / l
end

-- The frame's model-to-camera depth row, eye and sun in model space, kept
-- for the face pass.
local fa31, fa32, fa33 = 0, 0, 1
local emx_, emy_, emz_, lmx_, lmy_, lmz_ = 0, 0, 0, 0, 0, 1
local facesPending = false
-- Per face: its material's shade levels, or false for the translucent
-- disc (one lookup instead of the material, the disc test and the shade
-- table). Rebuilt after render.dirty() (a new mesh).
local FSH = {}
local FSB = {}                       -- the same for the underside (FBACK), or false
local fshStale = true
local faceLimit = 0                  -- faces in the model drawn this frame
local allInFront = true              -- no vertex behind the camera this frame
local LOD_PX = 35                    -- half span on screen below which the low detail model is used
local LOD_PX_LOW = 60                -- the same at low detail
local WEAK_DIST = 15                 -- metres beyond which one scale serves the whole model

local function modelFrame(h)
  modelMode = 0
  local dx, dy, dz = h.px - EYE_X, h.py - EYE_Y, h.pz - EYE_Z
  local t1 = dx * cRx + dy * cRy
  local t2 = dx * cUx + dy * cUy + dz * cUz
  local t3 = dx * cFx + dy * cFy + dz * cFz
  if t3 < 1.0 then return end
  local f = focal
  local radiusPx = model.SIZE * f / t3
  if radiusPx < 5 then
    modelMode = 1
    dotX, dotY = cx + t1 * f / t3, cy - t2 * f / t3
    dotR = radiusPx < 2 and 2 or radiusPx * 0.6
    return
  end
  modelMode = 2

  local r11, r12, r13 = h.r11, h.r12, h.r13
  local r21, r22, r23 = h.r21, h.r22, h.r23
  local r31, r32, r33 = h.r31, h.r32, h.r33
  -- M = camera * R: model axes in camera coordinates.
  local a11, a12, a13 = cRx * r11 + cRy * r21, cRx * r12 + cRy * r22, cRx * r13 + cRy * r23
  local a21 = cUx * r11 + cUy * r21 + cUz * r31
  local a22 = cUx * r12 + cUy * r22 + cUz * r32
  local a23 = cUx * r13 + cUy * r23 + cUz * r33
  local a31 = cFx * r11 + cFy * r21 + cFz * r31
  local a32 = cFx * r12 + cFy * r22 + cFz * r32
  local a33 = cFx * r13 + cFy * r23 + cFz * r33
  -- The eye and the sun in model space.
  local emx = -(r11 * dx + r21 * dy + r31 * dz)
  local emy = -(r12 * dx + r22 * dy + r32 * dz)
  local emz = -(r13 * dx + r23 * dy + r33 * dz)
  local lmx = r11 * SUN_X + r21 * SUN_Y + r31 * SUN_Z
  local lmy = r12 * SUN_X + r22 * SUN_Y + r32 * SUN_Z
  local lmz = r13 * SUN_X + r23 * SUN_Y + r33 * SUN_Z

  -- Small on screen: the low detail model (no canopy, spinner, gear or prop).
  local nv = model.vertexCount
  faceLimit = model.faceCount
  if radiusPx < (detail == 2 and LOD_PX or LOD_PX_LOW) then nv, faceLimit = model.coreVertices, model.coreFaces end

  local MX, MY, MZ = model.MX, model.MY, model.MZ
  local SX, SY = SX, SY
  local ccx, ccy = cx, cy
  allInFront = true
  if t3 > WEAK_DIST then
    -- Far away: one scale for the whole model (the perspective inside a
    -- 1.6 m model is too small to see), folded into the matrix.
    local k = f / t3
    local ox, oy = ccx + t1 * k, ccy - t2 * k
    local b11, b12, b13 = a11 * k, a12 * k, a13 * k
    local b21, b22, b23 = a21 * k, a22 * k, a23 * k
    for i = 1, nv do
      local x, y, z = MX[i], MY[i], MZ[i]
      SX[i] = ox + x * b11 + y * b12 + z * b13
      SY[i] = oy - x * b21 - y * b22 - z * b23
    end
  elseif t3 > NEAR + 2 then
    -- Nothing can be behind the camera.
    for i = 1, nv do
      local x, y, z = MX[i], MY[i], MZ[i]
      local k = f / (x * a31 + y * a32 + z * a33 + t3)
      SX[i] = ccx + (x * a11 + y * a12 + z * a13 + t1) * k
      SY[i] = ccy - (x * a21 + y * a22 + z * a23 + t2) * k
    end
  else
    allInFront = false
    for i = 1, nv do
      local x, y, z = MX[i], MY[i], MZ[i]
      local zc = x * a31 + y * a32 + z * a33 + t3
      if zc < NEAR then
        SX[i] = false
      else
        local k = f / zc
        SX[i] = ccx + (x * a11 + y * a12 + z * a13 + t1) * k
        SY[i] = ccy - (x * a21 + y * a22 + z * a23 + t2) * k
      end
    end
  end

  fa31, fa32, fa33 = a31, a32, a33
  emx_, emy_, emz_, lmx_, lmy_, lmz_ = emx, emy, emz, lmx, lmy, lmz
  facesPending = true
end

-- The face pass: cull faces turned away in model space, light the rest,
-- and sort them far to near.
local function modelFaces()
  facesPending = false
  local a31, a32, a33 = fa31, fa32, fa33
  local emx, emy, emz, lmx, lmy, lmz = emx_, emy_, emz_, lmx_, lmy_, lmz_
  local FA, FB, FC, FD, FTWO = model.FA, model.FB, model.FC, model.FD, model.FTWO
  local FNX, FNY, FNZ, FPD = model.FNX, model.FNY, model.FNZ, model.FPD
  local FCX, FCY, FCZ = model.FCX, model.FCY, model.FCZ
  if fshStale then
    fshStale = false
    local FMAT, FBACK, MAT_DISC = model.FMAT, model.FBACK, model.MAT_DISC
    for i = 1, model.faceCount do
      local m, b = FMAT[i], FBACK[i]
      FSH[i] = m ~= MAT_DISC and SHADES[m]
      FSB[i] = b and SHADES[b]
    end
    for i = model.faceCount + 1, #FSH do FSH[i], FSB[i] = nil, nil end
  end
  local discCol = C.disc
  local levels = SHADE_LEVELS - 1
  local inFront = allInFront
  local SX, KEY, FCOL, FSH, KF = SX, KEY, FCOL, FSH, KEY_FACES
  local n = 0
  for i = 1, faceLimit do
    local nx, ny, nz = FNX[i], FNY[i], FNZ[i]
    local facing = nx * emx + ny * emy + nz * emz - FPD[i]
    if facing > 0 or FTWO[i] then
      local d = FD[i]
      if inFront or (SX[FA[i]] and SX[FB[i]] and SX[FC[i]] and (d == 0 or SX[d])) then
        local sh = FSH[i]
        -- Seen from below, a wing or tailplane shows its underside.
        if facing < 0 then
          local b = FSB[i]
          if b then sh = b end
        end
        if sh then
          local lam = nx * lmx + ny * lmy + nz * lmz
          if facing < 0 then lam = -lam end
          if lam < 0 then lam = 0 end
          FCOL[i] = sh[(lam * levels + 1.5) // 1]
        else
          FCOL[i] = discCol
        end
        -- Far to near: a larger depth gives a smaller key. The face index
        -- rides in the low bits so a plain numeric sort (no Lua
        -- comparator) does the ordering.
        n = n + 1
        KEY[n] = ((8 - (FCX[i] * a31 + FCY[i] * a32 + FCZ[i] * a33)) * 4096) // 1 * KF + i
      end
    end
  end
  for i = n + 1, keyCount do KEY[i] = nil end
  keyCount = n
  if n > 1 then t_sort(KEY) end
end

---------------------------------------------------------------------------
-- Public.
---------------------------------------------------------------------------

function render.init(mesh)
  model = mesh
  buildColors()
  hillRing, treeRing = ringTables(HILLS), ringTables(TREELINE)
  t_sort(TREES, function(a, b)
    return a[1] * a[1] + a[2] * a[2] > b[1] * b[1] + b[2] * b[2]
  end)
  for i = 1, #TREES do
    local t = TREES[i]
    TREE_DIST[i] = sqrt(t[1] * t[1] + t[2] * t[2])
  end
end

-- Point the camera at the plane. zoomTarget is the wanted zoom; dt
-- smooths the motion.
function render.track(h, autoZoom, manualZoom, dt)
  local dx, dy, dz = h.px - EYE_X, h.py - EYE_Y, h.pz + 0.1 - EYE_Z
  local dh = sqrt(dx * dx + dy * dy)
  local yawT = atan(dx, dy)
  local pitchT = atan(dz, dh)
  -- Above the horizon, keep the horizon at the bottom of the view while the
  -- plane still fits, so there is always a ground reference; tilt up
  -- further only to keep the plane inside the top of the view.
  if pitchT > 0 then
    local halfV = atan(scrH * 0.5, focal)
    pitchT = max(min(pitchT, halfV * 0.8), pitchT - halfV * 0.35)
  end
  if pitchT > PITCH_MAX then pitchT = PITCH_MAX elseif pitchT < PITCH_MIN then pitchT = PITCH_MIN end
  local dyaw = yawT - camYaw
  if dyaw > pi then dyaw = dyaw - 2 * pi elseif dyaw < -pi then dyaw = dyaw + 2 * pi end
  local k = min(1, dt * 10)
  camYaw = camYaw + dyaw * k
  if camYaw > pi then camYaw = camYaw - 2 * pi elseif camYaw < -pi then camYaw = camYaw + 2 * pi end
  camPitch = camPitch + (pitchT - camPitch) * k
  local dist = sqrt(dh * dh + dz * dz)
  local zt = manualZoom or 1
  if autoZoom then zt = zt * min(3, max(1, dist / 20)) end
  zoom = zoom + (zt - zoom) * min(1, dt * 3)
end

-- Snap the camera to the plane (after a reset).
function render.snap(h, autoZoom, manualZoom)
  render.track(h, autoZoom, manualZoom, 10)
end

-- What the last frame was built from: when nothing has moved (the
-- plane sitting on the runway, the camera settled) the lists are kept.
local lastPX, lastPY, lastPZ, lastR11, lastR21, lastR13, lastR23, lastR33
local lastYaw, lastPitch, lastZoom, lastW, lastH, lastTop, lastKey
local EPS = 1e-4

local function unchanged(h, top, w, hgt, key)
  return w == lastW and hgt == lastH and top == lastTop and key == lastKey
    and abs(h.px - lastPX) < EPS and abs(h.py - lastPY) < EPS and abs(h.pz - lastPZ) < EPS
    and abs(h.r11 - lastR11) < EPS and abs(h.r21 - lastR21) < EPS and abs(h.r13 - lastR13) < EPS
    and abs(h.r23 - lastR23) < EPS and abs(h.r33 - lastR33) < EPS
    and abs(camYaw - lastYaw) < EPS and abs(camPitch - lastPitch) < EPS and abs(zoom - lastZoom) < EPS
end

-- Build the frame's display lists for a w x hgt scene whose top row is
-- top. animKey changes whenever the model's moving parts do. Returns false
-- (and keeps the last lists) when nothing has moved since the last frame.
function render.frame(h, top, w, hgt, animKey)
  if lastW and unchanged(h, top, w, hgt, animKey) then return false end
  lastPX, lastPY, lastPZ = h.px, h.py, h.pz
  lastR11, lastR21, lastR13, lastR23, lastR33 = h.r11, h.r21, h.r13, h.r23, h.r33
  lastYaw, lastPitch, lastZoom, lastW, lastH, lastTop, lastKey = camYaw, camPitch, zoom, w, hgt, top, animKey
  scrW, scrH, scrTop = w, hgt, top
  cx, cy = floor(w * 0.5), top + floor(hgt * 0.5)
  focal = (w * 0.5) / tan(HFOV * 0.5) * zoom
  kx = (w * 0.5 + 8) / focal
  ky = (hgt * 0.5 + 8) / focal

  local sy, cyw = sin(camYaw), cos(camYaw)
  local sp, cp = sin(camPitch), cos(camPitch)
  cFx, cFy, cFz = cp * sy, cp * cyw, sp
  cRx, cRy = cyw, -sy
  cUx, cUy, cUz = -sp * sy, -sp * cyw, cp

  -- Sky and ground bands, by elevation: the screen y of each edge (off the
  -- scene beyond the poles), and the band below it clamped to the scene.
  -- elevY() and band() written out.
  local bl = BANDS[detail]
  local R, BC = bl.R, bl.C
  local lim, f, ccy, st, sb = pi * 0.49, focal, cy, top, top + hgt
  local nb, yPrev = 0, nil
  for i = 1, #BC + 1 do
    local e = R[i] - camPitch
    local y
    if e >= lim then y = st - 1 elseif e <= -lim then y = sb + 1 else y = ccy - f * tan(e) end
    if yPrev and y > st and yPrev < sb and y > yPrev then
      local y0, y1 = yPrev, y
      if y0 < st then y0 = st end
      if y1 > sb then y1 = sb end
      y0 = y0 // 1
      nb = nb + 1
      BY[nb], BH[nb], BCOL[nb] = y0, y1 // 1 - y0 + 1, BC[i - 1]
    end
    yPrev = y
  end
  bandCount = nb

  -- Ground layer.
  triCount, lineCount = 0, 0
  if detail == 2 then
    silhouette(hillRing, C.hills)
    silhouette(treeRing, C.treeline)
    groundQuad(STRIP_X0, STRIP_Y0, STRIP_X1, STRIP_Y1, C.strip)
  end
  groundQuad(RUNWAY_X0, RUNWAY_Y0, RUNWAY_X1, RUNWAY_Y1, C.runway)
  -- Shadow under the plane: a cross, fuselage along the nose and wing
  -- across it.
  if h.pz < 30 then
    local nx, ny = h.r11, h.r21
    local nn = sqrt(nx * nx + ny * ny)
    if nn < 0.2 then nx, ny, nn = h.r13, h.r23, sqrt(h.r13 * h.r13 + h.r23 * h.r23) end
    nx, ny = nx / nn, ny / nn
    if detail == 2 then
      groundEllipse(h.px, h.py, nx, ny, 0.62, 0.10, 6, C.shadow)
      groundEllipse(h.px, h.py, nx, ny, 0.12, 0.75, 6, C.shadow)
    else
      -- A rhombus: nose, wing tips and tail.
      groundEllipse(h.px, h.py, nx, ny, 0.62, 0.75, 4, C.shadow)
    end
  end

  -- Field markings.
  if detail == 2 then
    local lc = C.line
    line(STRIP_X0, STRIP_Y0, 0, STRIP_X1, STRIP_Y0, 0, lc)
    line(STRIP_X0, STRIP_Y1, 0, STRIP_X1, STRIP_Y1, 0, lc)
  end
  -- Runway centre line, dashed.
  local mid = (RUNWAY_Y0 + RUNWAY_Y1) * 0.5
  for x = RUNWAY_X0 + 4, RUNWAY_X1 - 8, 12 do
    line(x, mid, 0, x + 5, mid, 0, C.runwayMark)
  end

  -- Trees, far to near, split around the plane.
  local hd = sqrt(h.px * h.px + h.py * h.py)
  treeCount, treeFar = 0, -1
  for i = 1, #TREES do
    if treeFar < 0 and TREE_DIST[i] < hd then treeFar = treeCount end
    tree(TREES[i])
  end
  if treeFar < 0 then treeFar = treeCount end

  modelFrame(h)
  if facesPending then modelFaces() end
  return true
end

-- Build the next frame even if nothing has moved (colours or settings
-- changed).
function render.dirty()
  lastW = nil
  fshStale = true
end

-- Detail level: 1 low (default), 2 high.
function render.setDetail(level)
  detail = (level == 2) and 2 or 1
  lastW = nil
end

local function drawTris(from, to)
  local last
  local dtri, color = lcd.drawFilledTriangle, lcd.color
  local X1, Y1, X2, Y2, X3, Y3, CO = TX1, TY1, TX2, TY2, TX3, TY3, TCOL
  for i = from, to do
    local col = CO[i]
    if col ~= last then color(col); last = col end
    dtri(X1[i], Y1[i], X2[i], Y2[i], X3[i], Y3[i])
  end
end

local function drawTrees(from, to)
  if from > to then return end
  local rect, circle, color = lcd.drawFilledRectangle, lcd.drawFilledCircle, lcd.color
  local trunk = C.trunk
  local KX, KY, KW, KH, RX, RY, RR, RCOL = KX, KY, KW, KH, RX, RY, RR, RCOL
  for i = from, to do
    local kh = KH[i]
    if kh > 0 then
      color(trunk)
      rect(KX[i], KY[i], KW[i], kh)
    end
    color(RCOL[i])
    circle(RX[i], RY[i], RR[i])
  end
end

-- Draw the scene, clipped to it (the plane is not clipped to the
-- view), into a w x h window.
function render.paint(w, h)
  lcd.setClipping(0, scrTop, w, scrH)
  local rect, color = lcd.drawFilledRectangle, lcd.color
  local BY, BH, BCOL = BY, BH, BCOL
  for i = 1, bandCount do
    color(BCOL[i])
    rect(0, BY[i], w, BH[i])
  end
  drawTris(1, triCount)
  local dline = lcd.drawLine
  local last
  local LX1, LY1, LX2, LY2, LCOL = LX1, LY1, LX2, LY2, LCOL
  for i = 1, lineCount do
    local col = LCOL[i]
    if col ~= last then color(col); last = col end
    dline(LX1[i], LY1[i], LX2[i], LY2[i])
  end
  drawTrees(1, treeFar)

  if modelMode == 2 then
    local FA, FB, FC, FD = model.FA, model.FB, model.FC, model.FD
    local dtri = lcd.drawFilledTriangle
    local SX, SY, KEY, FCOL, KF = SX, SY, KEY, FCOL, KEY_FACES
    last = nil
    for k = 1, keyCount do
      local i = KEY[k] % KF
      local col = FCOL[i]
      if col ~= last then color(col); last = col end
      local a, c = FA[i], FC[i]
      local ax, ay, cx3, cy3 = SX[a], SY[a], SX[c], SY[c]
      local b = FB[i]
      dtri(ax, ay, SX[b], SY[b], cx3, cy3)
      local d = FD[i]
      if d ~= 0 then dtri(ax, ay, cx3, cy3, SX[d], SY[d]) end
    end
  elseif modelMode == 1 then
    lcd.color(C.dot)
    lcd.drawFilledCircle(dotX, dotY, dotR)
  end

  drawTrees(treeFar + 1, treeCount)
  lcd.setClipping(0, 0, w, h)
end

-- Where the plane is on screen (for an off-screen arrow), and whether
-- it is drawn.
function render.modelScreen()
  return modelMode, dotX, dotY
end

function render.stats()
  return triCount, lineCount, keyCount
end

return render
