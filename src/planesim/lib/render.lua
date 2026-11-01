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

local SKY_E = {-2, 4, 14, 35, 92}                   -- band edges, degrees
local SKY_RGB = {{192, 216, 236}, {156, 194, 230}, {118, 166, 222}, {84, 136, 208}}
local GROUND_E = {0, -1.5, -8, -92}
local GROUND_RGB = {{94, 124, 80}, {82, 132, 60}, {76, 140, 56}}

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
  C.pine = {lcd.RGB(34, 82, 44), lcd.RGB(44, 96, 50), lcd.RGB(28, 70, 40)}
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
local kx, ky = 1, 1                  -- frustum slopes (with margin)

-- Bands: full-width rectangles.
local BY, BH, BCOL = {}, {}, {}
local bandCount = 0

-- Triangles: ground layer, then trees behind the plane, then trees in
-- front of it.
local TX1, TY1, TX2, TY2, TX3, TY3, TCOL = {}, {}, {}, {}, {}, {}, {}
local triCount, triGround, triFar = 0, 0, 0

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

local function band(y0, y1, col)
  if y1 <= 0 or y0 >= scrH or y1 <= y0 then return end
  if y0 < 0 then y0 = 0 end
  if y1 > scrH then y1 = scrH end
  local n = bandCount + 1
  bandCount = n
  BY[n], BH[n], BCOL[n] = floor(y0), floor(y1) - floor(y0) + 1, col
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

local function polygon(n, col)
  -- Outcodes: skip a polygon wholly outside one side of the view, project
  -- one wholly inside as it is, clip the rest.
  local any, all = 0, 31
  for i = 1, n do
    local x, y, z = PX[i], PY[i], PZ[i]
    local code = 0
    if z < NEAR then code = 1 end
    if x > kx * z then code = code | 2 elseif -x > kx * z then code = code | 4 end
    if y > ky * z then code = code | 8 elseif -y > ky * z then code = code | 16 end
    any, all = any | code, all & code
  end
  if all ~= 0 then return end
  if any == 0 then
    local f = focal
    local x0, y0 = cx + PX[1] * f / PZ[1], cy - PY[1] * f / PZ[1]
    local xp, yp = cx + PX[2] * f / PZ[2], cy - PY[2] * f / PZ[2]
    for i = 3, n do
      local xi, yi = cx + PX[i] * f / PZ[i], cy - PY[i] * f / PZ[i]
      tri(x0, y0, xp, yp, xi, yi, col)
      xp, yp = xi, yi
    end
    return
  end
  n = clipPlane(n, 0, 0, 1, -NEAR, PX, PY, PZ, QX, QY, QZ)
  n = clipPlane(n, -1, 0, kx, 0, QX, QY, QZ, PX, PY, PZ)
  n = clipPlane(n, 1, 0, kx, 0, PX, PY, PZ, QX, QY, QZ)
  n = clipPlane(n, 0, -1, ky, 0, QX, QY, QZ, PX, PY, PZ)
  n = clipPlane(n, 0, 1, ky, 0, PX, PY, PZ, QX, QY, QZ)
  if n < 3 then return end
  local f = focal
  local x0, y0 = cx + QX[1] * f / QZ[1], cy - QY[1] * f / QZ[1]
  local xp, yp = cx + QX[2] * f / QZ[2], cy - QY[2] * f / QZ[2]
  for i = 3, n do
    local xi, yi = cx + QX[i] * f / QZ[i], cy - QY[i] * f / QZ[i]
    tri(x0, y0, xp, yp, xi, yi, col)
    xp, yp = xi, yi
  end
end

local function groundQuad(x0, y0, x1, y1, col)
  PX[1], PY[1], PZ[1] = toCam(x0, y0, 0)
  PX[2], PY[2], PZ[2] = toCam(x1, y0, 0)
  PX[3], PY[3], PZ[3] = toCam(x1, y1, 0)
  PX[4], PY[4], PZ[4] = toCam(x0, y1, 0)
  polygon(4, col)
end

-- A flat ellipse on the ground: centre, half axes along a unit direction.
local function groundEllipse(gx, gy, ux, uy, ra, rb, n, col)
  for k = 1, n do
    local a = (k - 1) * 2 * pi / n
    local ca, sa = cos(a) * ra, sin(a) * rb
    PX[k], PY[k], PZ[k] = toCam(gx + ux * ca - uy * sa, gy + uy * ca + ux * sa, 0)
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
  local ax, ay, az = toCam(x0, y0, z0)
  local bx, by, bz = toCam(x1, y1, z1)
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

local function tree(t)
  local x, y, h, w = t[1], t[2], t[3], t[4]
  local bx, by, bz = toCam(x, y, 0)
  if bz < 2 then return end
  if abs(bx) > kx * bz + w then return end
  local tx, ty, tz = toCam(x, y, h)
  local f = focal
  local sbx, sby = cx + bx * f / bz, cy - by * f / bz
  local stx, sty = cx + tx * f / tz, cy - ty * f / tz
  local hw = w * 0.5 * f / bz
  local dx, dy = stx - sbx, sty - sby
  -- Trunk.
  local tw = max(1, hw * 0.14)
  local t1x, t1y = sbx + dx * 0.3, sby + dy * 0.3
  tri(sbx - tw, sby, sbx + tw, sby, t1x + tw, t1y, C.trunk)
  tri(sbx - tw, sby, t1x + tw, t1y, t1x - tw, t1y, C.trunk)
  -- Two canopy tiers.
  local col = C.pine[t[5]]
  local ax, ay = sbx + dx * 0.18, sby + dy * 0.18
  local mx, my = sbx + dx * 0.78, sby + dy * 0.78
  tri(ax - hw, ay, ax + hw, ay, mx, my, col)
  local bx2, by2 = sbx + dx * 0.48, sby + dy * 0.48
  local hw2 = hw * 0.72
  tri(bx2 - hw2, by2, bx2 + hw2, by2, stx, sty, col)
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
local faceLimit = 0                  -- faces in the model drawn this frame
local allInFront = true              -- no vertex behind the camera this frame
local LOD_PX = 35                    -- half span on screen below which the low detail model is used
local WEAK_DIST = 25                 -- metres beyond which one scale serves the whole model

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
  if radiusPx < LOD_PX then nv, faceLimit = model.coreVertices, model.coreFaces end

  local MX, MY, MZ = model.MX, model.MY, model.MZ
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
  local FA, FB, FC, FD, FMAT, FTWO = model.FA, model.FB, model.FC, model.FD, model.FMAT, model.FTWO
  local FBACK = model.FBACK
  local FNX, FNY, FNZ, FPD = model.FNX, model.FNY, model.FNZ, model.FPD
  local FCX, FCY, FCZ = model.FCX, model.FCY, model.FCZ
  local MAT_DISC = model.MAT_DISC
  local discCol = C.disc
  local levels = SHADE_LEVELS - 1
  local inFront = allInFront
  local n = 0
  for i = 1, faceLimit do
    local nx, ny, nz = FNX[i], FNY[i], FNZ[i]
    local facing = nx * emx + ny * emy + nz * emz - FPD[i]
    if facing > 0 or FTWO[i] then
      local d = FD[i]
      if inFront or (SX[FA[i]] and SX[FB[i]] and SX[FC[i]] and (d == 0 or SX[d])) then
        local mat = FMAT[i]
        -- Seen from below, a wing or tailplane shows its underside.
        if facing < 0 and FBACK[i] then mat = FBACK[i] end
        if mat == MAT_DISC then
          FCOL[i] = discCol
        else
          local lam = nx * lmx + ny * lmy + nz * lmz
          if facing < 0 then lam = -lam end
          if lam < 0 then lam = 0 end
          FCOL[i] = SHADES[mat][(lam * levels + 1.5) // 1]
        end
        -- Far to near: a larger depth gives a smaller key. The face index
        -- rides in the low bits so a plain numeric sort (no Lua
        -- comparator) does the ordering.
        n = n + 1
        KEY[n] = ((8 - (FCX[i] * a31 + FCY[i] * a32 + FCZ[i] * a33)) * 4096) // 1 * KEY_FACES + i
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
local lastYaw, lastPitch, lastZoom, lastW, lastH, lastKey
local EPS = 1e-4

local function unchanged(h, w, hgt, key)
  return w == lastW and hgt == lastH and key == lastKey
    and abs(h.px - lastPX) < EPS and abs(h.py - lastPY) < EPS and abs(h.pz - lastPZ) < EPS
    and abs(h.r11 - lastR11) < EPS and abs(h.r21 - lastR21) < EPS and abs(h.r13 - lastR13) < EPS
    and abs(h.r23 - lastR23) < EPS and abs(h.r33 - lastR33) < EPS
    and abs(camYaw - lastYaw) < EPS and abs(camPitch - lastPitch) < EPS and abs(zoom - lastZoom) < EPS
end

-- Build the frame's display lists for a w x h window. animKey changes
-- whenever the model's moving parts do. Returns false (and keeps the last
-- lists) when nothing has moved since the last frame.
function render.frame(h, w, hgt, animKey)
  if lastW and unchanged(h, w, hgt, animKey) then return false end
  lastPX, lastPY, lastPZ = h.px, h.py, h.pz
  lastR11, lastR21, lastR13, lastR23, lastR33 = h.r11, h.r21, h.r13, h.r23, h.r33
  lastYaw, lastPitch, lastZoom, lastW, lastH, lastKey = camYaw, camPitch, zoom, w, hgt, animKey
  scrW, scrH = w, hgt
  cx, cy = floor(w * 0.5), floor(hgt * 0.5)
  focal = (w * 0.5) / tan(HFOV * 0.5) * zoom
  kx = (w * 0.5 + 8) / focal
  ky = (hgt * 0.5 + 8) / focal

  local sy, cyw = sin(camYaw), cos(camYaw)
  local sp, cp = sin(camPitch), cos(camPitch)
  cFx, cFy, cFz = cp * sy, cp * cyw, sp
  cRx, cRy = cyw, -sy
  cUx, cUy, cUz = -sp * sy, -sp * cyw, cp

  -- Sky and ground bands, by elevation.
  bandCount = 0
  local f = focal
  for i = 1, #SKY_E - 1 do
    local e1 = rad(SKY_E[i + 1]) - camPitch
    local y1 = (e1 >= pi * 0.49) and -1 or (cy - f * tan(e1))
    local e0 = rad(SKY_E[i]) - camPitch
    local y0 = (e0 <= -pi * 0.49) and scrH or (cy - f * tan(e0))
    band(y1, y0, C.sky[i])
  end
  for i = 1, #GROUND_E - 1 do
    local e0 = rad(GROUND_E[i]) - camPitch
    local e1 = rad(GROUND_E[i + 1]) - camPitch
    local y0 = (e0 >= pi * 0.49) and -1 or (cy - f * tan(e0))
    local y1 = (e1 <= -pi * 0.49) and scrH + 1 or (cy - f * tan(e1))
    band(y0, y1, C.ground[i])
  end

  -- Ground layer.
  triCount, lineCount = 0, 0
  silhouette(hillRing, C.hills)
  silhouette(treeRing, C.treeline)
  groundQuad(STRIP_X0, STRIP_Y0, STRIP_X1, STRIP_Y1, C.strip)
  groundQuad(RUNWAY_X0, RUNWAY_Y0, RUNWAY_X1, RUNWAY_Y1, C.runway)
  -- Shadow under the plane: a cross, fuselage along the nose and wing
  -- across it.
  if h.pz < 30 then
    local nx, ny = h.r11, h.r21
    local nn = sqrt(nx * nx + ny * ny)
    if nn < 0.2 then nx, ny, nn = h.r13, h.r23, sqrt(h.r13 * h.r13 + h.r23 * h.r23) end
    nx, ny = nx / nn, ny / nn
    groundEllipse(h.px, h.py, nx, ny, 0.62, 0.10, 6, C.shadow)
    groundEllipse(h.px, h.py, nx, ny, 0.12, 0.75, 6, C.shadow)
  end
  triGround = triCount

  -- Field markings.
  local lc = C.line
  line(STRIP_X0, STRIP_Y0, 0, STRIP_X1, STRIP_Y0, 0, lc)
  line(STRIP_X0, STRIP_Y1, 0, STRIP_X1, STRIP_Y1, 0, lc)
  -- Runway centre line, dashed.
  local mid = (RUNWAY_Y0 + RUNWAY_Y1) * 0.5
  for x = RUNWAY_X0 + 4, RUNWAY_X1 - 8, 12 do
    line(x, mid, 0, x + 5, mid, 0, C.runwayMark)
  end

  -- Trees, far to near, split around the plane.
  local hd = sqrt(h.px * h.px + h.py * h.py)
  triFar = -1
  for i = 1, #TREES do
    if triFar < 0 and TREE_DIST[i] < hd then triFar = triCount end
    tree(TREES[i])
  end
  if triFar < 0 then triFar = triCount end

  modelFrame(h)
  if facesPending then modelFaces() end
  return true
end

-- Build the next frame even if nothing has moved (colours or settings
-- changed).
function render.dirty()
  lastW = nil
end

local function drawTris(from, to)
  local last
  local dtri = lcd.drawFilledTriangle
  for i = from, to do
    local col = TCOL[i]
    if col ~= last then lcd.color(col); last = col end
    dtri(TX1[i], TY1[i], TX2[i], TY2[i], TX3[i], TY3[i])
  end
end

function render.paint(w, h)
  local rect = lcd.drawFilledRectangle
  for i = 1, bandCount do
    lcd.color(BCOL[i])
    rect(0, BY[i], w, BH[i])
  end
  drawTris(1, triGround)
  local dline = lcd.drawLine
  local last
  for i = 1, lineCount do
    local col = LCOL[i]
    if col ~= last then lcd.color(col); last = col end
    dline(LX1[i], LY1[i], LX2[i], LY2[i])
  end
  drawTris(triGround + 1, triFar)

  if modelMode == 2 then
    local FA, FB, FC, FD = model.FA, model.FB, model.FC, model.FD
    local dtri = lcd.drawFilledTriangle
    last = nil
    for k = 1, keyCount do
      local i = KEY[k] % KEY_FACES
      local col = FCOL[i]
      if col ~= last then lcd.color(col); last = col end
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

  drawTris(triFar + 1, triCount)
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
