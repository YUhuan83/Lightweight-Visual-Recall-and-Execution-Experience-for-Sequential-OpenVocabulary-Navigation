-- Vehicle-local control lifetime. No geometry, collision or dynamics overrides.
local M = {version = 'wildos_control_lease_v1'}
local clock = HighPerfTimer()
local owner, timeoutMs, lastFeed, lastSequence = nil, 0, 0, 0
local tripped, reason, hasCommand, holding = false, nil, false, false
local physicsElapsed = 0

local function finite(x)
  return type(x) == 'number' and x == x and x > -math.huge and x < math.huge
end

local function tokenValid(token)
  return type(token) == 'string' and #token == 32 and token:match('^[0-9a-f]+$')
end

local function nowMs()
  local now = clock:getTime()
  assert(finite(now), 'Invalid vehicle lease clock')
  return now
end

local function brakeValues()
  -- These are the ordinary input values consumed by the stock controllers.
  -- Setting them also covers physics steps when graphics input processing lags.
  electrics.values.throttle, electrics.values.throttle_input = 0, 0
  electrics.values.brake, electrics.values.brake_input = 1, 1
  electrics.values.parkingbrake, electrics.values.parkingbrake_input = 1, 1
  electrics.values.steering_input = 0
end

local function brakeInputs()
  assert(FILTER_DIRECT == 2, 'Unsupported direct input filter')
  input.event('throttle', 0, FILTER_DIRECT)
  input.event('brake', 1, FILTER_DIRECT)
  input.event('parkingbrake', 1, FILTER_DIRECT)
  input.event('steering', 0, FILTER_DIRECT)
  input.event('clutch', 0, FILTER_DIRECT)
  brakeValues()
end

local function trip(why)
  tripped, reason, holding = true, why, true
  electrics.values.wildosControlLeaseTripped = 1
  brakeInputs()
end

local function checkExpiry()
  if owner == nil then return end
  local age = nowMs() - lastFeed
  if not tripped and (age < 0 or age >= timeoutMs) then
    trip(age < 0 and 'clock_regression' or 'command_expired')
  end
end

local function stationary()
  local speed = obj:getVelocity():length()
  return finite(speed) and speed < 0.05
end

function M.claim(token, timeoutSec)
  assert(tokenValid(token), 'Invalid lease owner')
  assert(finite(timeoutSec) and timeoutSec > 0 and timeoutSec <= 2, 'Invalid lease timeout')
  assert(FILTER_DIRECT == 2 and FILTER_PAD == 1, 'Unsupported input filter contract')
  checkExpiry()
  if owner == token then
    assert(not tripped and not holding and timeoutMs == timeoutSec * 1000, 'Owner cannot rearm its expired lease')
    return 'wildos_lease_claimed_v1' -- Idempotent claim never renews time.
  end
  assert(owner == nil or tripped, 'Another control owner is active')
  assert(stationary(), 'Vehicle must stop before a new control owner')
  owner, timeoutMs, lastFeed, lastSequence = token, timeoutSec * 1000, nowMs(), 0
  tripped, reason, hasCommand, holding = false, nil, false, false
  physicsElapsed = 0
  electrics.values.wildosControlLeaseTripped = 0
  brakeInputs() -- Hold until the first validated command.
  return 'wildos_lease_claimed_v1'
end

function M.feed(token, sequence)
  assert(token == owner and owner ~= nil, 'Vehicle control owner mismatch')
  assert(finite(sequence) and sequence % 1 == 0 and sequence > lastSequence
         and sequence <= 9007199254740991, 'Old or invalid vehicle control sequence')
  checkExpiry() -- A delayed request cannot revive an expired lease.
  assert(not tripped and not holding, 'Vehicle control lease expired or closing')
  lastFeed, lastSequence, hasCommand = nowMs(), sequence, true
end

function M.release(token)
  assert(token == owner and owner ~= nil, 'Vehicle control owner mismatch')
  brakeInputs()
  if not stationary() then
    holding = true -- Remain guarded after the client disconnects.
    return 'wildos_lease_brake_held_v1'
  end
  owner, tripped, reason, hasCommand, holding = nil, false, nil, false, false
  electrics.values.wildosControlLeaseTripped = 0
  return 'wildos_lease_released_v1'
end

function M.status()
  -- Read only: diagnostics must not create the watchdog effect being tested.
  return {version=M.version, owner=owner, active=owner ~= nil, tripped=tripped,
          reason=reason, holding=holding, last_sequence=lastSequence,
          timeout_sec=timeoutMs/1000, age_sec=owner and (nowMs()-lastFeed)/1000 or nil}
end

function M.updateGFX()
  checkExpiry()
  if owner and (tripped or holding or not hasCommand) then brakeInputs() end
end

function M.onPhysicsStep(dt)
  if owner == nil then return end
  physicsElapsed = physicsElapsed + dt
  if physicsElapsed < 0.02 then return end
  physicsElapsed = 0
  checkExpiry()
  if tripped or holding or not hasCommand then brakeValues() end
end

function M.onReset()
  if owner then trip('vehicle_reset') end
end

function M.onExtensionUnloaded()
  if owner then brakeInputs() end
end

return M
