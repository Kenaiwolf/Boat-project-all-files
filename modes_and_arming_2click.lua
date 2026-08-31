-- modes_and_arming_2click.lua    
-- CH3 double-press (2 confirmed transitions within DOUBLE_PRESS_WINDOW_MS) -> HOLD    
-- CH4 double-press (2 confirmed transitions within DOUBLE_PRESS_WINDOW_MS) -> GUIDED    
-- CH5 double-press (2 confirmed transitions within DOUBLE_PRESS_WINDOW_MS) -> ACRO    
-- CH6 double-press (2 confirmed transitions within DOUBLE_PRESS_WINDOW_MS) -> MANUAL    
--
-- Rover mode numbers (Rover/Parameters.cpp MODE1 @Values):  
--   0:Manual 1:Acro 3:Steering 4:Hold 15:Guided  
local MODE_HOLD    = 4  
local MODE_GUIDED  = 15  
local MODE_ACRO    = 1  
local MODE_MANUAL  = 0  
  
local CH_HOLD   = 3    
local CH_GUIDED = 4    
local CH_ACRO   = 5    
local CH_MANUAL = 6    
  
-- ---- Rudder-gated arm/disarm (CH3 double-press opens a window) ----  
local rcmap_roll_p = Parameter()  
rcmap_roll_p:init('RCMAP_ROLL')  
local ROLL_CHANNEL = math.floor(rcmap_roll_p:get())  -- rudder/steering channel  
  
local ARM_WINDOW_MS   = 5000  -- rudder gesture must START within this many ms of CH3 double-press  
local ARM_HOLD_MS     = 2000  -- rudder must be held past threshold this long, continuously, to complete  
local RUDDER_THRESH = 4000/4500   -- match stock threshold exactly (~0.8889)  
  
local arm_window_deadline = nil  -- ms deadline for gesture to START; nil = window closed  
local arm_gesture_start_ms = nil -- ms when current continuous deflection began; nil = not deflected  
local arm_gesture_dir = nil      -- 1 = right(arm), -1 = left(disarm) for the in-progress gesture
  
local PWM_HIGH_THRESH = 1800  -- above this = HIGH    
local PWM_LOW_THRESH  = 1200  -- below this = LOW    
  
-- ---- Double-press confirmation ----  
local CONFIRM_TICKS          = 4     -- consecutive identical ticks (~200ms @20Hz) to accept a state as real (debounce)    
local CONFIRM_TICKS_MANUAL   = 2     -- shorter debounce (~100ms) for CH6's fast momentary press, so short presses aren't missed    
local DOUBLE_PRESS_WINDOW_MS = 1000  -- 2nd confirmed transition must land within this window of the 1st
  
-- classify raw pwm into "HIGH", "LOW", or nil (mid/invalid -> ignore)  
local function classify(pwm)  
  if pwm == nil then  
    return nil  
  end  
  if pwm >= PWM_HIGH_THRESH then  
    return "HIGH"  
  elseif pwm <= PWM_LOW_THRESH then  
    return "LOW"  
  end  
  return nil -- in the dead zone between thresholds, ignore  
end  
  
-- per-channel debounce state  
local pending_state    = {}  -- candidate state currently being confirmed  
local pending_count    = {}  -- consecutive ticks pending_state has held  
local confirmed_state  = {}  -- last debounced, accepted state (nil until first confirmed)  
  
-- per-channel double-press state  
local transition_count = {}  -- 0 or 1 confirmed transitions seen in current window  
local window_start_ms  = {}  -- timestamp of the 1st confirmed transition in the window  
  
-- returns true only once a NEW state has been read identically for  
-- CONFIRM_TICKS consecutive ticks (filters boot latch / bounce / single  
-- corrupted frames). The very first confirmed state after boot/script  
-- start is a baseline, never itself reported as a transition.  
local function confirmed_transition(chan, ticks)    
  local confirm_ticks = ticks or CONFIRM_TICKS    
  local pwm = rc:get_pwm(chan)    
  local raw = classify(pwm)    
    
  if raw == nil then    
    -- invalid/mid-range/no data: reset debounce, don't touch confirmed_state    
    pending_count[chan] = 0    
    return false, nil    
  end    
    
  if pending_state[chan] == raw then    
    pending_count[chan] = (pending_count[chan] or 0) + 1    
  else    
    pending_state[chan] = raw    
    pending_count[chan] = 1    
  end    
    
  if pending_count[chan] < confirm_ticks then    
    return false, nil    
  end    
    
  if confirmed_state[chan] == nil then    
    -- first stable reading ever seen: set baseline only, not a transition    
    confirmed_state[chan] = raw    
    return false, nil    
  end    
    
  if raw ~= confirmed_state[chan] then    
    confirmed_state[chan] = raw    
    return true, raw    
  end    
    
  return false, nil    
end
  
-- returns true only when TWO confirmed transitions on this channel have  
-- occurred within DOUBLE_PRESS_WINDOW_MS of each other. A single confirmed  
-- transition (e.g. the one-shot artifact from RX reconnect/boot) only  
-- starts the window and is otherwise discarded if a 2nd never follows.  
local function double_press_detected(chan, ticks, rising_only)    
  local transitioned, new_state = confirmed_transition(chan, ticks)    
  if not transitioned then    
    return false    
  end    
    
  -- for momentary switches (press=HIGH, auto-release=LOW): only count the    
  -- press (rising) edge as a user action; ignore the auto-release (falling)    
  -- edge so one physical press doesn't count as two events    
  if rising_only and new_state ~= "HIGH" then    
    return false    
  end    
    
  local now = millis()    
  local ws  = window_start_ms[chan]    
    
  if (transition_count[chan] or 0) == 0 or ws == nil or (now - ws) > DOUBLE_PRESS_WINDOW_MS then    
    -- this is the 1st confirmed transition of a new window    
    transition_count[chan] = 1    
    window_start_ms[chan]  = now    
    return false    
  end    
    
  -- this is the 2nd confirmed transition within the window -> accept    
  transition_count[chan] = 0    
  window_start_ms[chan]  = nil    
  return true    
end
  
-- checks rudder deflection against the open arm/disarm window opened by  
-- a CH3 double-press. The 5s window only gates the START of a deflection;  
-- once a deflection begins before the deadline, the 2s continuous hold is  
-- allowed to finish even if that pushes past the 5s mark.  
local function check_rudder_arm_disarm()  
  local now = millis()  
  
  if arm_window_deadline == nil then  
    return -- no window open, nothing to do  
  end  
  
  local roll_ch = rc:get_channel(ROLL_CHANNEL)  
  local norm = roll_ch and roll_ch:norm_input_dz() or 0  
  
  local dir = 0  
  if norm >= RUDDER_THRESH then  
    dir = 1  
  elseif norm <= -RUDDER_THRESH then  
    dir = -1  
  end  
  
  if arm_gesture_start_ms == nil then  
    -- no deflection currently in progress  
    if dir == 0 then  
      if now > arm_window_deadline then  
        arm_window_deadline = nil -- window expired, nothing ever started  
      end  
      return  
    end  
    -- a deflection is starting: only allowed if window is still open  
    if now > arm_window_deadline then  
      arm_window_deadline = nil  
      return  
    end  
    arm_gesture_start_ms = now  
    arm_gesture_dir = dir  
    return  
  end  
  
  -- a deflection is already in progress  
  if dir ~= arm_gesture_dir then  
    -- released or reversed before completing the hold: abort this attempt  
    arm_gesture_start_ms = nil  
    arm_gesture_dir = nil  
    if now > arm_window_deadline then  
      arm_window_deadline = nil  
    end  
    return  
  end  
  
  if (now - arm_gesture_start_ms) >= ARM_HOLD_MS then  
    -- held long enough: act, regardless of whether we're past the 5s deadline  
    if arm_gesture_dir == 1 then  
      if not arming:is_armed() then  
        if arming:arm() then  
          gcs:send_text(6, "Lua: CH3+rudder-right -> ARMED")  
        end  
      end  
    else  
      if arming:is_armed() then  
        if arming:disarm() then  
          gcs:send_text(6, "Lua: CH3+rudder-left -> DISARMED")  
        end  
      end  
    end  
    arm_window_deadline = nil  
    arm_gesture_start_ms = nil  
    arm_gesture_dir = nil  
  end  
end  
  
function update()    
  check_rudder_arm_disarm()  
  
  if double_press_detected(CH_HOLD) then      
    if vehicle:set_mode(MODE_HOLD) then      
      gcs:send_text(6, "Lua: CH3 double-press -> HOLD")      
      arm_window_deadline = millis() + ARM_WINDOW_MS  
      arm_gesture_start_ms = nil  
      arm_gesture_dir = nil  
    end      
  end    
    
  if double_press_detected(CH_GUIDED) then    
    if vehicle:set_mode(MODE_GUIDED) then    
      gcs:send_text(6, "Lua: CH4 double-press -> GUIDED")    
    end    
  end    
    
  if double_press_detected(CH_ACRO) then    
    if vehicle:set_mode(MODE_ACRO) then    
      gcs:send_text(6, "Lua: CH5 double-press -> ACRO")    
    end    
  end    
    
  if double_press_detected(CH_MANUAL, CONFIRM_TICKS_MANUAL, true) then      
    if vehicle:set_mode(MODE_MANUAL) then      
      gcs:send_text(6, "Lua: CH6 double-press -> MANUAL")      
    end      
  end
  
  return update, 50 -- run at 20Hz  
end  
  
return update, 1000 -- first run after 1s (let RC input stabilize)