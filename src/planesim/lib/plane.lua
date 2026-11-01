-- The aeroplane mesh: a low-poly Extra-style aerobatic plane.
--
-- The geometry comes from wingflight-lua-ethos-suite app/plane3d.lua (its
-- default airframe, itself a copy of the Wingflight Configurator's
-- model.gltf): red body, dark ailerons, elevator and rudder, white spinner
-- and spatted gear, a canopy and a two-blade prop. Here the wings and
-- tailplane are white underneath, so upright and inverted read apart at a
-- distance, and the prop turns into a translucent disc under power.
--
-- Sizes are written in the glTF file's units (x = span, + is the left seen
-- from behind; y = up; z = aft) and converted at build to the sim's body
-- axes in metres: X = forward (nose), Y = left, Z = up, origin near the
-- centre of gravity. The span becomes 2 m (a large aerobatic model).
--
-- Built in stages (plane.buildStep() until it returns true) because Ethos
-- stops a callback that runs too many Lua instructions. Each face gets a
-- precomputed unit normal, centroid and plane offset, so the renderer can
-- cull and light faces in model space. plane.animate() moves the control
-- surfaces and swaps the prop between blades and disc.

local cos, sin, sqrt, pi = math.cos, math.sin, math.sqrt, math.pi
local min = math.min

local plane = {}

local S = 1.0 / 3.55        -- glTF units -> metres (2 m span)
local Z0, Y0 = -0.05, 0.0   -- glTF point that becomes the body origin

local function bx(gz) return -(gz - Z0) * S end
local function by(gx) return gx * S end
local function bz(gy) return (gy - Y0) * S end

-- Materials. A translucent material (the prop disc) is drawn unshaded.
plane.MAT_BODY = 1
plane.MAT_SURFACE = 2
plane.MAT_WHITE = 3
plane.MAT_CANOPY = 4
plane.MAT_PROP = 5
plane.MAT_DISC = 6
local MAT_BODY, MAT_SURFACE, MAT_WHITE, MAT_CANOPY, MAT_PROP, MAT_DISC = 1, 2, 3, 4, 5, 6

local MX, MY, MZ = {}, {}, {}
plane.MX, plane.MY, plane.MZ = MX, MY, MZ
plane.vertexCount = 0
-- The low detail model: the first coreVertices vertices and coreFaces faces
-- (fuselage, wings, tail).
plane.coreVertices, plane.coreFaces = 0, 0

-- Faces: up to four vertex indices (FD = 0 for a triangle), the material,
-- the material seen from behind (FBACK, plates only: the wing undersides),
-- whether it is a plate seen from both sides, and the precomputed normal,
-- centroid and plane offset.
local FA, FB, FC, FD, FMAT, FBACK, FTWO = {}, {}, {}, {}, {}, {}, {}
local FNX, FNY, FNZ, FCX, FCY, FCZ, FPD = {}, {}, {}, {}, {}, {}, {}
plane.FA, plane.FB, plane.FC, plane.FD, plane.FMAT, plane.FBACK, plane.FTWO = FA, FB, FC, FD, FMAT, FBACK, FTWO
plane.FNX, plane.FNY, plane.FNZ = FNX, FNY, FNZ
plane.FCX, plane.FCY, plane.FCZ, plane.FPD = FCX, FCY, FCZ, FPD
plane.faceCount = 0

-- Contact points for the physics (body axes, metres).
plane.WHEEL_X, plane.WHEEL_Y, plane.WHEEL_Z = bx(-0.50), by(0.76), bz(-1.06)
plane.TAIL_X, plane.TAIL_Z = bx(3.60), bz(-0.06)
plane.TIP_X, plane.TIP_Y, plane.TIP_Z = bx(0.20), by(3.55), bz(-0.06)
plane.NOSE_X, plane.NOSE_Z = bx(-2.04), bz(0.05)
plane.PROP_X, plane.PROP_Z, plane.PROP_R = bx(-1.70), bz(0.05), 0.95 * S
plane.FIN_X, plane.FIN_Z = bx(3.30), bz(1.25)
plane.CANOPY_X, plane.CANOPY_Z = bx(0.85), bz(0.76)
plane.SPAN = 2 * 3.55 * S
plane.SIZE = 3.55 * S      -- half span: the size the renderer judges detail by

local vertexCount, faceCount = 0, 0

local function vtx(gx, gy, gz)
  vertexCount = vertexCount + 1
  MX[vertexCount], MY[vertexCount], MZ[vertexCount] = bx(gz), by(gx), bz(gy)
  return vertexCount
end

local function pushFace(a, b, c, d, mat, twoSided, back)
  faceCount = faceCount + 1
  FA[faceCount], FB[faceCount], FC[faceCount], FD[faceCount] = a, b, c, d or 0
  FMAT[faceCount] = mat
  FBACK[faceCount] = back or false
  FTWO[faceCount] = twoSided
  return faceCount
end

-- A flat plate seen from both sides; back is the material of its underside
-- (the build turns its normal to point up).
local function plate(a, b, c, d, mat, back)
  return pushFace(a, b, c, d, mat, true, back)
end

-- One face of a closed part; (rx, ry, rz) is a glTF point inside it, used
-- to turn the face outwards.
local function solid(a, b, c, d, mat, rx, ry, rz)
  local ix, iy, iz = bx(rz), by(rx), bz(ry)
  local ux, uy, uz = MX[b] - MX[a], MY[b] - MY[a], MZ[b] - MZ[a]
  local vx, vy, vz = MX[c] - MX[a], MY[c] - MY[a], MZ[c] - MZ[a]
  local nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
  local n = d and 4 or 3
  local gx = (MX[a] + MX[b] + MX[c] + (d and MX[d] or 0)) / n - ix
  local gy = (MY[a] + MY[b] + MY[c] + (d and MY[d] or 0)) / n - iy
  local gz = (MZ[a] + MZ[b] + MZ[c] + (d and MZ[d] or 0)) / n - iz
  if nx * gx + ny * gy + nz * gz < 0 then
    if d then a, b, c, d = a, d, c, b else b, c = c, b end
  end
  pushFace(a, b, c, d, mat, false)
end

---------------------------------------------------------------------------
-- Control surfaces: hinge h1-h2, trailing edge t1/t2 with their neutral
-- positions, so animate() can swing them.
---------------------------------------------------------------------------

local SKIND, SSIDE, SFACE, SH1, SH2, ST1, ST2 = {}, {}, {}, {}, {}, {}, {}
local RX, RY, RZ = {}, {}, {}
local surfaceCount = 0

local function surface(kind, side, h1, h2, t2, t1)
  local f = plate(h1, h2, t2, t1, MAT_SURFACE)
  surfaceCount = surfaceCount + 1
  SKIND[surfaceCount], SSIDE[surfaceCount], SFACE[surfaceCount] = kind, side, f
  SH1[surfaceCount], SH2[surfaceCount], ST1[surfaceCount], ST2[surfaceCount] = h1, h2, t1, t2
  RX[t1], RY[t1], RZ[t1] = MX[t1], MY[t1], MZ[t1]
  RX[t2], RY[t2], RZ[t2] = MX[t2], MY[t2], MZ[t2]
end

---------------------------------------------------------------------------
-- Parts
---------------------------------------------------------------------------

-- Fuselage stations: {z, half width, bottom y, top y, side y}.
local BODY = {
  {-1.62, 0.36, -0.20, 0.30, 0.05},
  {-1.00, 0.54, -0.42, 0.38, 0.00},
  {-0.30, 0.60, -0.46, 0.46, 0.00},
  { 1.00, 0.50, -0.30, 0.50, 0.05},
  { 1.80, 0.30, -0.22, 0.62, 0.12},
  { 3.55, 0.05, -0.04, 0.40, 0.18},
}

-- Each station becomes a ring of six points around the body; band k runs
-- from point k to point k + 1, and band 4 (the bottom) is the white belly.
local BAND_MATS = {MAT_BODY, MAT_BODY, MAT_BODY, MAT_WHITE, MAT_BODY, MAT_BODY}

local function fuselage()
  local ring = {}
  for i = 1, #BODY do
    local s = BODY[i]
    local z, w, yb, yt, ym = s[1], s[2], s[3], s[4], s[5]
    local t = w * 0.65
    ring[i] = {vtx(-t, yt, z), vtx(t, yt, z), vtx(w, ym, z), vtx(t, yb, z), vtx(-t, yb, z), vtx(-w, ym, z)}
  end
  for i = 1, #BODY - 1 do
    local r0, r1 = ring[i], ring[i + 1]
    local rz = (BODY[i][1] + BODY[i + 1][1]) * 0.5
    local ry = (BODY[i][5] + BODY[i + 1][5]) * 0.5
    for k = 1, 6 do
      local k2 = (k % 6) + 1
      solid(r0[k], r0[k2], r1[k2], r1[k], BAND_MATS[k], 0, ry, rz)
    end
  end
  local f, l = ring[1], ring[#BODY]
  local first, last = BODY[1], BODY[#BODY]
  solid(f[1], f[2], f[3], f[4], MAT_BODY, 0, first[5], first[1] + 1)
  solid(f[4], f[5], f[6], f[1], MAT_BODY, 0, first[5], first[1] + 1)
  solid(l[1], l[2], l[3], l[4], MAT_BODY, 0, last[5], last[1] - 1)
  solid(l[4], l[5], l[6], l[1], MAT_BODY, 0, last[5], last[1] - 1)
end

-- Straight wing: root and tip as {x, leading edge z, trailing edge z, y};
-- ailerons from 30% of the half span to the tip.
local ROOT = {0.55, -0.55, 1.00, -0.12}
local TIP = {3.55, -0.33, 0.76, -0.06}
local AILERON_FROM = 0.30

local function wingAt(t)
  local x = t * TIP[1]
  local u = (x - ROOT[1]) / (TIP[1] - ROOT[1])
  if u < 0 then u = 0 elseif u > 1 then u = 1 end
  local le = ROOT[2] + (TIP[2] - ROOT[2]) * u
  local te = ROOT[3] + (TIP[3] - ROOT[3]) * u
  local y = ROOT[4] + (TIP[4] - ROOT[4]) * u
  local chord = te - le
  local hinge = te - min(0.25 + 0.1 * chord, 0.8 * chord)
  return x, y, le, hinge, te
end

-- One wing (side 1: +x, the left seen from behind; side 2: the right): an
-- inner panel and an outer panel with the aileron behind it.
local function wing(side)
  local sx = (side == 1) and 1 or -1
  local cuts = {ROOT[1] / TIP[1], AILERON_FROM, 1}
  local px, py, ple, ph, pte = wingAt(cuts[1])
  for i = 2, 3 do
    local x, y, le, hinge, te = wingAt(cuts[i])
    plate(vtx(px * sx, py, ple), vtx(x * sx, y, le), vtx(x * sx, y, hinge), vtx(px * sx, py, ph), MAT_BODY, MAT_WHITE)
    local h1, h2 = vtx(px * sx, py, ph), vtx(x * sx, y, hinge)
    local t2, t1 = vtx(x * sx, y, te), vtx(px * sx, py, pte)
    if i == 3 then
      surface("aileron", side, h1, h2, t2, t1)
    else
      plate(h1, h2, t2, t1, MAT_BODY, MAT_WHITE)
    end
    px, py, ple, ph, pte = x, y, le, hinge, te
  end
end

local function tail()
  for side = 1, 2 do
    local s = (side == 1) and 1 or -1
    plate(vtx(0.10 * s, 0.36, 2.92), vtx(1.55 * s, 0.36, 3.08), vtx(1.55 * s, 0.36, 3.47), vtx(0.10 * s, 0.36, 3.47), MAT_BODY, MAT_WHITE)
    surface("elevator", side, vtx(0.04 * s, 0.36, 3.47), vtx(1.60 * s, 0.36, 3.47), vtx(1.60 * s, 0.36, 3.80), vtx(0.04 * s, 0.36, 3.86))
  end
  -- Fin and rudder.
  plate(vtx(0, 0.48, 2.85), vtx(0, 1.25, 3.28), vtx(0, 1.25, 3.52), vtx(0, 0.28, 3.52), MAT_BODY)
  surface("rudder", 1, vtx(0, -0.02, 3.52), vtx(0, 1.25, 3.52), vtx(0, 1.25, 3.84), vtx(0, -0.02, 4.14))
end

local function canopy()
  local cF = vtx(0, 0.47, 0.20)
  local cBL, cBR = vtx(0.27, 0.49, 0.80), vtx(-0.27, 0.49, 0.80)
  local cTL, cTR = vtx(0.16, 0.76, 0.85), vtx(-0.16, 0.76, 0.85)
  local cR = vtx(0, 0.62, 1.75)
  solid(cF, cBR, cTR, nil, MAT_CANOPY, 0, 0.55, 0.9)
  solid(cF, cTR, cTL, nil, MAT_CANOPY, 0, 0.55, 0.9)
  solid(cF, cTL, cBL, nil, MAT_CANOPY, 0, 0.55, 0.9)
  solid(cBR, cR, cTR, nil, MAT_CANOPY, 0, 0.55, 0.9)
  solid(cTR, cR, cTL, nil, MAT_CANOPY, 0, 0.55, 0.9)
  solid(cTL, cR, cBL, nil, MAT_CANOPY, 0, 0.55, 0.9)
end

local function spinner()
  local tip = vtx(0, 0.05, -2.04)
  local base = {}
  for k = 1, 6 do
    local a = (k - 1) * pi / 3
    base[k] = vtx(0.2 * cos(a), 0.05 + 0.2 * sin(a), -1.64)
  end
  for k = 1, 6 do
    solid(tip, base[k], base[(k % 6) + 1], nil, MAT_WHITE, 0, 0.05, -1.75)
  end
end

local function gear()
  for side = 1, 2 do
    local s = (side == 1) and 1 or -1
    local px = 0.76 * s
    plate(vtx(0.25 * s, -0.40, -0.52), vtx(px, -0.82, -0.54), vtx(px, -0.82, -0.38), vtx(0.25 * s, -0.40, -0.25), MAT_WHITE)
    local front, rear = vtx(px, -0.93, -0.92), vtx(px, -0.90, -0.10)
    local top, bottom = vtx(px, -0.77, -0.50), vtx(px, -1.06, -0.50)
    local out, inn = vtx(px + 0.08 * s, -0.92, -0.50), vtx(px - 0.08 * s, -0.92, -0.50)
    solid(front, top, out, nil, MAT_WHITE, px, -0.92, -0.50)
    solid(front, out, bottom, nil, MAT_WHITE, px, -0.92, -0.50)
    solid(front, bottom, inn, nil, MAT_WHITE, px, -0.92, -0.50)
    solid(front, inn, top, nil, MAT_WHITE, px, -0.92, -0.50)
    solid(rear, out, top, nil, MAT_WHITE, px, -0.92, -0.50)
    solid(rear, bottom, out, nil, MAT_WHITE, px, -0.92, -0.50)
    solid(rear, inn, bottom, nil, MAT_WHITE, px, -0.92, -0.50)
    solid(rear, top, inn, nil, MAT_WHITE, px, -0.92, -0.50)
  end
end

-- The prop: two blades, and a translucent disc for when it is turning.
-- Whichever is not shown has its vertices folded onto the hub.
local propFirst, propLast, bladeLast = 0, 0, 0
local PRX, PRY, PRZ = {}, {}, {}
local hubX, hubY, hubZ = 0, 0, 0

local function prop()
  local hub = vtx(0, 0.05, -1.70)
  hubX, hubY, hubZ = MX[hub], MY[hub], MZ[hub]
  propFirst = vertexCount + 1
  plate(vtx(0.12, 0.11, -1.70), vtx(0.95, 0.08, -1.70), vtx(0.95, 0.02, -1.70), vtx(0.12, -0.01, -1.70), MAT_PROP)
  plate(vtx(-0.12, 0.11, -1.70), vtx(-0.95, 0.08, -1.70), vtx(-0.95, 0.02, -1.70), vtx(-0.12, -0.01, -1.70), MAT_PROP)
  bladeLast = vertexCount
  local first = vertexCount + 1
  local n = 8
  for k = 1, n do
    local a = (k - 1) * 2 * pi / n
    vtx(0.95 * cos(a), 0.05 + 0.95 * sin(a), -1.70)
  end
  for k = 0, n - 1 do
    plate(hub, first + k, first + ((k + 1) % n), nil, MAT_DISC)
  end
  propLast = vertexCount
  for v = propFirst, propLast do PRX[v], PRY[v], PRZ[v] = MX[v], MY[v], MZ[v] end
end

---------------------------------------------------------------------------
-- Normals
---------------------------------------------------------------------------

local function finishFace(i)
  local a, b, c, d = FA[i], FB[i], FC[i], FD[i]
  local nx, ny, nz, cx, cy, cz
  if d ~= 0 then
    local ux, uy, uz = MX[c] - MX[a], MY[c] - MY[a], MZ[c] - MZ[a]
    local vx, vy, vz = MX[d] - MX[b], MY[d] - MY[b], MZ[d] - MZ[b]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    cx = (MX[a] + MX[b] + MX[c] + MX[d]) * 0.25
    cy = (MY[a] + MY[b] + MY[c] + MY[d]) * 0.25
    cz = (MZ[a] + MZ[b] + MZ[c] + MZ[d]) * 0.25
  else
    local ux, uy, uz = MX[b] - MX[a], MY[b] - MY[a], MZ[b] - MZ[a]
    local vx, vy, vz = MX[c] - MX[a], MY[c] - MY[a], MZ[c] - MZ[a]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    cx = (MX[a] + MX[b] + MX[c]) / 3
    cy = (MY[a] + MY[b] + MY[c]) / 3
    cz = (MZ[a] + MZ[b] + MZ[c]) / 3
  end
  local len = sqrt(nx * nx + ny * ny + nz * nz)
  if len > 0 then nx, ny, nz = nx / len, ny / len, nz / len end
  FNX[i], FNY[i], FNZ[i] = nx, ny, nz
  FCX[i], FCY[i], FCZ[i] = cx, cy, cz
  FPD[i] = nx * cx + ny * cy + nz * cz
end

local function finishFaces()
  for i = 1, faceCount do
    -- A plate with an underside: turn it so its front faces up.
    if FBACK[i] then
      local a, b, c, d = FA[i], FB[i], FC[i], FD[i]
      local ux, uy = MX[c] - MX[a], MY[c] - MY[a]
      local vx, vy
      if d ~= 0 then vx, vy = MX[d] - MX[b], MY[d] - MY[b] else vx, vy = MX[b] - MX[a], MY[b] - MY[a] end
      local nz = (d ~= 0) and (ux * vy - uy * vx) or (vx * uy - vy * ux)
      if nz < 0 then
        if d ~= 0 then FB[i], FD[i] = d, b else FB[i], FC[i] = c, b end
      end
    end
    finishFace(i)
  end
end

---------------------------------------------------------------------------
-- Animation
---------------------------------------------------------------------------

-- Swing surface k's trailing edge by angle (radians) about its hinge,
-- towards (dx, dy, dz) for a positive angle.
local function deflect(k, angle, dx, dy, dz)
  local h1, h2 = SH1[k], SH2[k]
  local ax, ay, az = MX[h2] - MX[h1], MY[h2] - MY[h1], MZ[h2] - MZ[h1]
  local al = sqrt(ax * ax + ay * ay + az * az)
  ax, ay, az = ax / al, ay / al, az / al
  local t = ST1[k]
  local ux, uy, uz = RX[t] - MX[h1], RY[t] - MY[h1], RZ[t] - MZ[h1]
  local nx, ny, nz = ay * uz - az * uy, az * ux - ax * uz, ax * uy - ay * ux
  local nl = sqrt(nx * nx + ny * ny + nz * nz)
  if nl > 0 then nx, ny, nz = nx / nl, ny / nl, nz / nl end
  if nx * dx + ny * dy + nz * dz < 0 then nx, ny, nz = -nx, -ny, -nz end
  local c, s = cos(angle), sin(angle)
  for j = 1, 2 do
    local v = (j == 1) and ST1[k] or ST2[k]
    local px, py, pz = RX[v] - MX[h1], RY[v] - MY[h1], RZ[v] - MZ[h1]
    local along = px * ax + py * ay + pz * az
    local rx, ry, rz = px - ax * along, py - ay * along, pz - az * along
    local r = sqrt(rx * rx + ry * ry + rz * rz)
    MX[v] = MX[h1] + ax * along + rx * c + nx * r * s
    MY[v] = MY[h1] + ay * along + ry * c + ny * r * s
    MZ[v] = MZ[h1] + az * along + rz * c + nz * r * s
  end
  finishFace(SFACE[k])
end

local DEFLECT = 0.45          -- radians at full stick (~26 deg)
local lastAil, lastEle, lastRud, lastSpin

-- Move the surfaces with the sticks (-1..1: aileron > 0 rolls right,
-- elevator > 0 pushes the nose down, rudder > 0 yaws right) and show the
-- prop as blades or a disc.
function plane.animate(ail, ele, rud, spinning)
  if ail ~= lastAil or ele ~= lastEle or rud ~= lastRud then
    lastAil, lastEle, lastRud = ail, ele, rud
    for k = 1, surfaceCount do
      local kind = SKIND[k]
      if kind == "aileron" then
        -- Rolling right: the left aileron (side 1) goes down, the right up.
        local a = ail * DEFLECT
        deflect(k, (SSIDE[k] == 1) and -a or a, 0, 0, 1)
      elseif kind == "elevator" then
        deflect(k, -ele * DEFLECT, 0, 0, 1)
      else
        deflect(k, rud * DEFLECT, 0, -1, 0)
      end
    end
  end
  if spinning ~= lastSpin then
    lastSpin = spinning
    for v = propFirst, propLast do
      local show = (v > bladeLast) == spinning
      if show then
        MX[v], MY[v], MZ[v] = PRX[v], PRY[v], PRZ[v]
      else
        MX[v], MY[v], MZ[v] = hubX, hubY, hubZ
      end
    end
  end
end

---------------------------------------------------------------------------
-- Building
---------------------------------------------------------------------------

local oldVertices, oldFaces = 0, 0

local STAGES = {
  function()
    oldVertices, oldFaces = vertexCount, faceCount
    vertexCount, faceCount, surfaceCount = 0, 0, 0
    fuselage()
  end,
  function()
    wing(1)
    wing(2)
    tail()
    -- Up to here is the low detail model.
    plane.coreVertices, plane.coreFaces = vertexCount, faceCount
  end,
  function()
    canopy()
    spinner()
    gear()
    prop()
    for i = vertexCount + 1, oldVertices do MX[i], MY[i], MZ[i] = nil, nil, nil end
    for i = faceCount + 1, oldFaces do
      FA[i], FB[i], FC[i], FD[i], FMAT[i], FBACK[i], FTWO[i] = nil, nil, nil, nil, nil, nil, nil
      FNX[i], FNY[i], FNZ[i], FCX[i], FCY[i], FCZ[i], FPD[i] = nil, nil, nil, nil, nil, nil, nil
    end
    plane.vertexCount, plane.faceCount = vertexCount, faceCount
  end,
  function()
    finishFaces()
    lastAil, lastSpin = nil, nil
    plane.animate(0, 0, 0, false)
  end,
}

local stage = 0
plane.ready = false

function plane.build()
  stage = 1
  plane.ready = false
end

-- One build stage; true once the mesh is complete.
function plane.buildStep()
  if stage == 0 then return plane.ready end
  STAGES[stage]()
  if stage < #STAGES then
    stage = stage + 1
    return false
  end
  stage = 0
  plane.ready = true
  return true
end

return plane
