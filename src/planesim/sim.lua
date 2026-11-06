-- PlaneSim: fly a 3D aerobatic plane with the radio's sticks.
--
-- The pilot stands at the edge of a flying field; the plane sits at the
-- west end of the runway in front, pointing east. Lower the throttle to arm
-- the motor, then take off. Enter puts the plane back on the runway, Page
-- opens the settings, Exit closes the tool.
--
-- wakeup does all the work (sticks, physics, camera, the frame's display
-- lists); paint only replays the lists and draws the HUD text.

local plane = assert(loadfile("lib/plane.lua"))()
local physics = assert(loadfile("lib/physics.lua"))()
local render = assert(loadfile("lib/render.lua"))()

local floor, sqrt, min, max, rad = math.floor, math.sqrt, math.min, math.max, math.rad

local sim = {}

local STEP = 0.02                -- physics step (s)
local MAX_STEPS = 5              -- per wakeup; a longer gap slows time instead
local RANGE_LIMIT = 300          -- metres from the pilot
-- The HUD text sits on opaque bars above (and, with the stats, below) the
-- scene, so it needs no shadow and is only drawn every other frame:
-- between, only the scene is invalidated and Ethos keeps the bars.
local BAR_H, STATS_H = 22, 40
local TOO_CLOSE = 1.5

-- Analog sources: the sticks by function, as Ethos numbers them.
local MEMBER_RUD, MEMBER_ELE, MEMBER_THR, MEMBER_AIL = 0, 1, 2, 3

local CONFIG_FILE = "planesim.cfg"

-- How fast the rates follow the sticks (1/s).
local RESPONSES = {8, 14, 24, 40}
local RESPONSE_NAMES = {"Soft", "Normal", "Crisp", "Locked in"}

local DEFAULTS = {
  level = false,      -- self-level (true) or rate
  rollRate = 360,     -- deg/s at full stick and full airspeed
  pitchRate = 240,
  yawRate = 180,
  expo = 30,          -- percent
  response = 3,       -- index into RESPONSES
  autoZoom = true,
  detail = 1,         -- 1 low (flat backdrop, simple model), 2 high
  stats = false,
}
local SAVE_ORDER = {"level", "rollRate", "pitchRate", "yawRate", "expo", "response", "autoZoom", "detail", "stats"}

---------------------------------------------------------------------------
-- Helpers
---------------------------------------------------------------------------

local function now()
  return os.clock()
end

local function clamp(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi end
  return v
end

local function scriptDir()
  if not (debug and debug.getinfo) then return "" end
  local ok, info = pcall(debug.getinfo, 1, "S")
  if not ok or not info or type(info.source) ~= "string" then return "" end
  local source = info.source
  if source:sub(1, 1) == "@" then source = source:sub(2) end
  return source:match("^(.*[/\\])") or ""
end

local SCRIPT_DIR = scriptDir()

local function configPaths()
  return {SCRIPT_DIR .. CONFIG_FILE, "/scripts/planesim/" .. CONFIG_FILE, CONFIG_FILE}
end

local function loadConfig()
  local cfg = {}
  for k, v in pairs(DEFAULTS) do cfg[k] = v end
  if not (io and io.open) then return cfg end
  local f
  for _, p in ipairs(configPaths()) do
    f = io.open(p, "r")
    if f then break end
  end
  if not f then return cfg end
  local lines = {}
  while true do
    local ok, line = pcall(f.read, f, "*l")
    if not ok or not line then break end
    lines[#lines + 1] = line
  end
  f:close()
  for _, line in ipairs(lines) do
    local key, value = line:match("^%s*([%w_]+)%s*=%s*(.-)%s*$")
    local d = key and DEFAULTS[key]
    if type(d) == "boolean" then
      cfg[key] = (value == "1" or value == "true")
    elseif type(d) == "number" then
      cfg[key] = tonumber(value) or d
    end
  end
  cfg.rollRate = clamp(floor(cfg.rollRate), 30, 1000)
  cfg.pitchRate = clamp(floor(cfg.pitchRate), 30, 1000)
  cfg.yawRate = clamp(floor(cfg.yawRate), 30, 1000)
  cfg.expo = clamp(floor(cfg.expo), 0, 90)
  if not RESPONSES[cfg.response] then cfg.response = DEFAULTS.response end
  if cfg.detail ~= 2 then cfg.detail = 1 end
  return cfg
end

local function saveConfig(cfg)
  if not (io and io.open) then return end
  local f
  for _, p in ipairs(configPaths()) do
    f = io.open(p, "w")
    if f then break end
  end
  if not f then return end
  for _, key in ipairs(SAVE_ORDER) do
    local v = cfg[key]
    if type(v) == "boolean" then v = v and "1" or "0" end
    f:write(key, "=", tostring(v), "\n")
  end
  f:close()
end

local function analogSource(member)
  if not (system and system.getSource) then return nil end
  local ok, src = pcall(system.getSource, {category = CATEGORY_ANALOG, member = member})
  if ok then return src end
  return nil
end

-- A stick in -1..1 (sources give -1024..1024).
local function stick(src)
  if not src then return 0 end
  local v = src:value()
  if type(v) ~= "number" then return 0 end
  v = v / 1024
  if v > 1 then return 1 elseif v < -1 then return -1 end
  return v
end

local function shape(v, expo)
  if v > -0.02 and v < 0.02 then return 0 end
  return v * (1 - expo) + v * v * v * expo
end

local function keyIs(value, ...)
  for i = 1, select("#", ...) do
    local k = select(i, ...)
    if k ~= nil and value == k then return true end
  end
  return false
end

---------------------------------------------------------------------------
-- State
---------------------------------------------------------------------------

local function resetPlane(state)
  physics.reset(state.h, render.START_X, render.START_Y, render.START_HEADING)
  state.phase = "arm"
  state.flightTime = 0
  render.snap(state.h, state.cfg.autoZoom, state.manualZoom)
  render.dirty()
end

local function formatHud(state)
  if not state.hudLeftDirty then return end
  state.hudLeftDirty = false
  state.hudLeft = string.format("ALT %.1fm  SPD %.1fm/s  THR %d%%",
    state.hudAlt / 10, state.hudSpd / 10, floor(state.hudThr))
end

local function setFont(f)
  if f then lcd.font(f) end
end

-- HUD strings, rebuilt only when what they show changes. Returns whether
-- the bar texts changed and whether the centre message (drawn over the
-- scene) changed. The flight data line changes nearly every frame in
-- flight, so it is only formatted when the bar is drawn (formatHud).
local function updateHud(state)
  local h = state.h
  local changed = false
  local up = h.pz - 0.22
  if up < 0 then up = 0 end
  local alt = (up * 10 + 0.5) // 1
  local spd = (h.airspeed * 10 + 0.5) // 1
  local thr = (h.thr * 100 + 0.5) // 1
  if alt ~= state.hudAlt or spd ~= state.hudSpd or thr ~= state.hudThr then
    state.hudAlt, state.hudSpd, state.hudThr = alt, spd, thr
    state.hudLeftDirty = true
    changed = true
  end
  local secs = floor(state.flightTime)
  if secs ~= state.hudSecs then
    state.hudSecs = secs
    state.hudTime = string.format("%d:%02d", floor(secs / 60), secs % 60)
    changed = true
  end
  local level = state.cfg.level
  if level ~= state.hudLevel then
    state.hudLevel = level
    state.hudMode = level and "Self-level" or "Rate"
    changed = true
  end

  local key = (state.phase == "arm") and 1 or (state.phase == "crashed") and 2 or 0
  if key ~= state.hudMsgKey then
    state.hudMsgKey = key
    if key == 1 then
      state.message, state.subMessage = "Throttle low to arm", "Enter: reset   Page: settings"
    elseif key == 2 then
      state.message, state.subMessage = "CRASH: " .. (h.crashReason or ""), "Enter: back to the runway"
    else
      state.message, state.subMessage = nil, nil
    end
    return changed, true
  end
  return changed, false
end

---------------------------------------------------------------------------
-- Settings
---------------------------------------------------------------------------

local function closeSettings(state)
  state.settingsOpen = false
  pcall(form.clear)
  state.lastTime = nil
  state.paintScene, state.paintBar = true, true
  lcd.invalidate()
end

local function openSettings(state)
  if not (form and form.clear and form.addLine and form.addChoiceField) then return end
  form.clear()
  state.settingsOpen = true
  local cfg = state.cfg
  local function set(key, v)
    cfg[key] = v
    saveConfig(cfg)
  end

  form.addChoiceField(form.addLine("Flight mode"), nil, {{"Rate", 0}, {"Self-level", 1}},
    function() return cfg.level and 1 or 0 end, function(v) set("level", v == 1) end)
  local function number(label, lo, hi, step, suffix, key)
    local f = form.addNumberField(form.addLine(label), nil, lo, hi,
      function() return cfg[key] end, function(v) set(key, v) end)
    if f then
      if f.step then f:step(step) end
      if f.suffix then f:suffix(suffix) end
      if f.default then f:default(DEFAULTS[key]) end
    end
  end
  number("Roll rate", 30, 1000, 10, " deg/s", "rollRate")
  number("Pitch rate", 30, 1000, 10, " deg/s", "pitchRate")
  number("Yaw rate", 30, 1000, 10, " deg/s", "yawRate")
  number("Expo", 0, 90, 5, "%", "expo")
  local responseChoices = {}
  for i = 1, #RESPONSES do responseChoices[i] = {RESPONSE_NAMES[i], i} end
  form.addChoiceField(form.addLine("Response"), nil, responseChoices,
    function() return cfg.response end, function(v) set("response", v) end)
  -- Low detail is for the radio, where the screen fills set the frame rate.
  form.addChoiceField(form.addLine("Detail"), nil, {{"Low", 1}, {"High", 2}},
    function() return cfg.detail end,
    function(v)
      set("detail", v)
      render.setDetail(v)
    end)
  if form.addBooleanField then
    form.addBooleanField(form.addLine("Auto zoom"), nil,
      function() return cfg.autoZoom end, function(v) set("autoZoom", v) end)
    form.addBooleanField(form.addLine("Show CPU stats"), nil,
      function() return cfg.stats end, function(v) set("stats", v) end)
  end
  local back = function() closeSettings(state) end
  if form.addButton then
    form.addButton(form.addLine(""), nil, {text = "Back to flying", press = back})
  elseif form.addTextButton then
    form.addTextButton(form.addLine(""), nil, "Back to flying", back)
  end
end

---------------------------------------------------------------------------
-- Callbacks
---------------------------------------------------------------------------

function sim.create()
  local state = {
    cfg = loadConfig(),
    h = {},
    view = {},           -- the plane as drawn (position and attitude)
    inp = {roll = 0, pitch = 0, yaw = 0, thr = 0, rollRate = 0, pitchRate = 0, yawRate = 0,
      response = 14, level = false},
    phase = "boot",
    manualZoom = 1,
    acc = 0,
    flightTime = 0,
    nextAwake = 0,
    -- Stats: counters for the current second, and the last second's values.
    wakes = 0, paints = 0, tickSum = 0, paintSum = 0, rateAt = 0,
    rate = 0, fps = 0, tickMs = 0, paintMs = 0,
    -- What the next paint draws: the scene, the bars; barDirty: the bars
    -- changed since they were last drawn.
    paintScene = true, paintBar = true, barDirty = true, frameNo = 0,
    hudLeft = "", hudTime = "0:00", hudMode = "",
    srcRud = analogSource(MEMBER_RUD),
    srcEle = analogSource(MEMBER_ELE),
    srcThr = analogSource(MEMBER_THR),
    srcAil = analogSource(MEMBER_AIL),
  }
  plane.build()
  physics.setGeometry(plane)
  render.init(plane)
  render.setDetail(state.cfg.detail)
  physics.reset(state.h, render.START_X, render.START_Y, render.START_HEADING)
  return state
end

local function keepAwake(state, t)
  if t < state.nextAwake then return end
  state.nextAwake = t + 1
  if system.resetBacklightTimeout then pcall(system.resetBacklightTimeout) end
  if system.resetFocusTimeout then pcall(system.resetFocusTimeout) end
end

local function usage()
  return system.getInstructionsUsage and system.getInstructionsUsage() or 0
end

function sim.wakeup(state)
  if state.settingsOpen then return end
  local t = now()
  keepAwake(state, t)

  if state.phase == "boot" then
    if plane.buildStep() then resetPlane(state) end
    lcd.invalidate()
    return
  end

  local cfg = state.cfg
  if cfg.stats then
    state.wakes = state.wakes + 1
    if t >= state.rateAt then
      -- Once a second: wakeups and paints per second, mean wakeup and
      -- paint time (ms).
      local wakes = state.wakes
      state.rate, state.fps = wakes, state.paints
      state.tickMs = state.tickSum * 1000 / wakes
      state.paintMs = state.paints > 0 and state.paintSum * 1000 / state.paints or 0
      state.wakes, state.paints, state.tickSum, state.paintSum = 0, 0, 0, 0
      state.rateAt = t + 1
    end
  end

  local dt = state.lastTime and (t - state.lastTime) or 0
  state.lastTime = t
  if dt < 0 then dt = 0 elseif dt > MAX_STEPS * STEP then dt = MAX_STEPS * STEP end

  local h, inp = state.h, state.inp
  local expo = cfg.expo / 100
  local thr = stick(state.srcThr)
  state.thr = thr
  inp.roll = shape(stick(state.srcAil), expo)
  inp.pitch = shape(stick(state.srcEle), expo)
  inp.yaw = shape(stick(state.srcRud), expo)
  inp.rollRate = rad(cfg.rollRate)
  inp.pitchRate = rad(cfg.pitchRate)
  inp.yawRate = rad(cfg.yawRate)
  inp.response = RESPONSES[cfg.response]
  inp.level = cfg.level

  if state.phase == "arm" then
    inp.thr = 0
    if thr < -0.8 then state.phase = "fly" end
  else
    inp.thr = (thr + 1) * 0.5
  end

  if state.phase ~= "crashed" then
    state.acc = state.acc + dt
    local n = 0
    while state.acc >= STEP and n < MAX_STEPS do
      physics.step(h, inp, STEP)
      state.acc = state.acc - STEP
      n = n + 1
    end
    if not h.landed then state.flightTime = state.flightTime + dt end
    local d2 = h.px * h.px + h.py * h.py
    if d2 > RANGE_LIMIT * RANGE_LIMIT then
      h.crashed, h.crashReason = true, "Out of range"
    elseif d2 + (h.pz - 1.7) * (h.pz - 1.7) < TOO_CLOSE * TOO_CLOSE then
      h.crashed, h.crashReason = true, "Too close to the pilot!"
    end
    if h.crashed then
      state.phase = "crashed"
      if system.playTone then pcall(system.playTone, 300, 400, 0) end
    end
  end

  -- Control surfaces follow the sticks; the prop blurs into a disc when
  -- the motor runs.
  local spinning = h.thr > 0.04 and not h.crashed
  plane.animate(inp.roll, inp.pitch, inp.yaw, spinning)
  local animKey = inp.roll * 1e6 + inp.pitch * 1e3 + inp.yaw + (spinning and 0.5 or 0)

  -- Draw the plane where it is now, not where the last physics step
  -- left it: a frame holds a varying number of 20 ms steps (3 or 4 at
  -- 13 fps), and drawing the stepped state made fast passes stutter. The
  -- time not stepped yet (state.acc, under one step) is carried forward
  -- with the velocity and the body rates, for the picture only.
  local view, a = state.view, state.acc
  view.px, view.py, view.pz = h.px + h.vx * a, h.py + h.vy * a, h.pz + h.vz * a
  local wx, wy, wz = h.p * a, h.q * a, h.r * a
  local r11, r12, r13 = h.r11, h.r12, h.r13
  local r21, r22, r23 = h.r21, h.r22, h.r23
  local r31, r32, r33 = h.r31, h.r32, h.r33
  view.r11, view.r12, view.r13 = r11 + r12 * wz - r13 * wy, r12 - r11 * wz + r13 * wx, r13 + r11 * wy - r12 * wx
  view.r21, view.r22, view.r23 = r21 + r22 * wz - r23 * wy, r22 - r21 * wz + r23 * wx, r23 + r21 * wy - r22 * wx
  view.r31, view.r32, view.r33 = r31 + r32 * wz - r33 * wy, r32 - r31 * wz + r33 * wx, r33 + r31 * wy - r32 * wx

  local w, wh = lcd.getWindowSize()
  local sceneH = wh - BAR_H - (cfg.stats and STATS_H or 0)
  render.track(view, cfg.autoZoom, state.manualZoom, dt)
  local drew = render.frame(view, BAR_H, w, sceneH, animKey)
  local barChanged, msgChanged = updateHud(state)
  if barChanged or cfg.stats then state.barDirty = true end

  -- Repaint only what changed: the scene, and the bars every other frame
  -- (every frame when the scene is still). Both bars with a still scene
  -- (the stats on the pad) repaint the whole window.
  local scene = drew or msgChanged
  local bar = false
  if state.barDirty then
    local n = state.frameNo + 1
    state.frameNo = n
    bar = not scene or n % 2 == 0
  end
  if bar then
    state.barDirty = false
    if cfg.stats then scene = true end
  end
  if cfg.stats then state.tickSum = state.tickSum + (now() - t) end
  if not (scene or bar) then return end
  state.paintScene, state.paintBar = scene, bar
  -- Invalidate rectangles only: a plain invalidate() makes Ethos clear the
  -- window before the paint (measured on an X20: 1 fps at High detail, 2-3
  -- at Low), and the scene repaints every pixel of its rectangle anyway.
  if scene and bar then
    lcd.invalidate(0, 0, w, wh)
  elseif scene then
    lcd.invalidate(0, BAR_H, w, sceneH)
  else
    lcd.invalidate(0, 0, w, BAR_H)
  end
end

local function shadowText(x, y, text, flags)
  lcd.color(BLACK)
  lcd.drawText(x + 1, y + 1, text, flags)
  lcd.color(WHITE)
  lcd.drawText(x, y, text, flags)
end

function sim.paint(state)
  if state.settingsOpen then return end
  local w, h = lcd.getWindowSize()
  if state.phase == "boot" then
    lcd.color(lcd.RGB(30, 40, 60))
    lcd.drawFilledRectangle(0, 0, w, h)
    setFont(FONT_L)
    shadowText(w // 2, h // 2 - 12, "PlaneSim: loading", CENTERED)
    return
  end
  local stats = state.cfg.stats
  local t0 = stats and now()
  -- A paint Ethos asked for by itself draws everything.
  local scene, bar = state.paintScene, state.paintBar
  if not (scene or bar) then scene, bar = true, true end
  state.paintScene, state.paintBar = false, false

  if scene then
    render.paint(w, h)
    if state.message then
      local y = BAR_H + floor(h * 0.14)
      setFont(FONT_L)
      shadowText(w // 2, y, state.message, CENTERED)
      if state.subMessage then
        setFont(FONT_S)
        shadowText(w // 2, y + 30, state.subMessage, CENTERED)
      end
    end
  end

  if bar then
    lcd.color(BLACK)
    lcd.drawFilledRectangle(0, 0, w, BAR_H)
    setFont(FONT_S)
    lcd.color(WHITE)
    formatHud(state)
    lcd.drawText(6, 2, state.hudLeft)
    lcd.drawText(w // 2, 2, state.hudMode, CENTERED)
    lcd.drawText(w - 6, 2, state.hudTime, RIGHT)
    if stats then
      -- Per second: paints (fps), wakeups, mean wakeup and paint time; the
      -- paint's instruction budget (Ethos 26.1.3+, last frame).
      local y = h - STATS_H
      lcd.color(BLACK)
      lcd.drawFilledRectangle(0, y, w, STATS_H)
      lcd.color(WHITE)
      local tris, lines, faces = render.stats()
      local inp = state.inp
      lcd.drawText(6, y + 1, string.format("fps %d  wake %d/s  tick %.1f ms  paint %.1f ms  budget %d%%",
        state.fps, state.rate, state.tickMs, state.paintMs, state.cpuPaint or 0))
      lcd.drawText(6, y + 20, string.format("tri %d  faces %d  mem %dk  R%+d E%+d T%+d A%+d",
        tris, faces, floor(collectgarbage("count")),
        floor(inp.yaw * 100), floor(inp.pitch * 100), floor((state.thr or 0) * 100), floor(inp.roll * 100)))
    end
  end

  if stats then
    if system.getInstructionsUsage then state.cpuPaint = usage() end
    state.paints = state.paints + 1
    state.paintSum = state.paintSum + (now() - t0)
  end
end

-- Page keys, with the values Ethos sends as a fallback.
local PAGE_KEYS = {KEY_PAGE_BREAK or 32, KEY_PAGE_LONG or 128}

function sim.event(state, category, value)
  if state.settingsOpen then
    if category == EVT_CLOSE or (category == EVT_KEY and keyIs(value, KEY_EXIT_BREAK, KEY_EXIT_FIRST)) then
      closeSettings(state)
      -- Exit arrives as FIRST then BREAK: the BREAK would close the tool.
      state.swallowExit = (value == KEY_EXIT_FIRST)
      return true
    end
    return false
  end
  if category ~= EVT_KEY then return false end

  if state.swallowExit and keyIs(value, KEY_EXIT_BREAK) then
    state.swallowExit = false
    return true
  end
  state.swallowExit = false

  -- Enter acts on the press: once a settings form has been open, Ethos no
  -- longer passes the Enter release (KEY_ENTER_BREAK) to the tool.
  if keyIs(value, KEY_ENTER_FIRST) then
    if state.phase ~= "boot" then resetPlane(state) end
    return true
  end
  if keyIs(value, KEY_ENTER_BREAK, KEY_ENTER_LONG) then
    return true
  end
  if keyIs(value, PAGE_KEYS[1], PAGE_KEYS[2]) then
    openSettings(state)
    return true
  end
  if keyIs(value, KEY_PAGE_FIRST or 96) then
    return true
  end
  if keyIs(value, KEY_ROTARY_RIGHT) then
    state.manualZoom = min(3, state.manualZoom * 1.1)
    return true
  end
  if keyIs(value, KEY_ROTARY_LEFT) then
    state.manualZoom = max(0.5, state.manualZoom / 1.1)
    return true
  end
  return false
end

function sim.close(state)
  if state and state.settingsOpen then pcall(form.clear) end
  collectgarbage("collect")
end

return sim
