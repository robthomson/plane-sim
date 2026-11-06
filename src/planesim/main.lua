-- PlaneSim system tool entry point. Without the title bar (title = false)
-- the tool gets the whole screen, and the frame rate is a little higher.

local sim = assert(loadfile("sim.lua"))()

local function init()
  local ok, icon = pcall(lcd.loadMask, "gfx/icon.png")
  system.registerSystemTool({
    name = "PlaneSim",
    icon = ok and icon or nil,
    title = false,
    create = sim.create,
    wakeup = sim.wakeup,
    event = sim.event,
    paint = sim.paint,
    close = sim.close,
  })
end

return {init = init}
