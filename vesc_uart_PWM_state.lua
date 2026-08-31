-- vesc_uart_PWM_state.lua  
-- VESC UART control via COMM_CUSTOM_APP_DATA (ID=36)  
local PORT = serial:find_serial(0)  
if not PORT then  
  gcs:send_text(3,"VESC: no port")  
  return  
end  
  
local function crc16(s)  
  local c=0  
  for i=1,#s do  
    c=c~(s:byte(i)<<8)  
    for _=1,8 do  
      if c&0x8000~=0 then c=(c<<1)~0x1021  
      else c=c<<1 end  
      c=c&0xFFFF  
    end  
  end  
  return c  
end  
  
local function pkt(payload)  
  local d=string.char(0x24)..payload  
  local c=crc16(d)  
  return string.char(2,#d)..d..string.char(c>>8,c&0xFF,3)  
end  
  
local TIMEOUT=3000    
local POLL=5    
local STEER_TRIM=1500    
local STEER_HALF_RANGE=500  -- = (SERVOx_MAX - SERVOx_MIN)/2 for steering channel    
  
local THROTTLE_FUNC=70            -- k_throttle  
local THROTTLE_SAFE_PWM=1500      -- confirmed neutral/stop PWM  
local THROTTLE_OVERRIDE_MS=100    -- re-applied every tick (POLL=5ms), short timeout so a script stall fails safe  
local THROTTLE_CHAN=SRV_Channels:find_channel(THROTTLE_FUNC)  -- 0-indexed, resolved once at startup  
if not THROTTLE_CHAN then    
  gcs:send_text(3,"VESC: no throttle chan found")    
end
local qry_div=0  
local last_valid_pkt=millis():tofloat()  
local rx_buf=""  
local vesc_st=-1
local STATE_NAMES={[0]="IDLE",[1]="HOMING",[2]="ACTIVE",[3]="STOWED",[4]="FAILSAFE"}
local function state_name(n)  
  return STATE_NAMES[n] or ("UNK("..tostring(n)..")")  
end  
local connected=false        
local was_armed=false        
local was_hold=true        
local throttle_override_active=false      
local last_auth_fail_reason=nil    -- tracks previous pre-arm gate outcome, so set_aux_auth_* only fires on change   
  
-- pre-arm gate: block arming until VESC has reported ACTIVE at least once,  
-- and surface the reason to the GCS via the standard aux-auth mechanism  
local auth_id = arming:get_aux_auth_id()   
  
local function update()  
  local now=millis():tofloat()  
  local armed=arming:is_armed()  
  local mode=vehicle:get_mode()  
  local hold=(mode==4)  -- Rover HOLD  
  
  local cmd_sent=false    
  if armed and was_armed and was_hold and not hold then
    PORT:writestring(pkt(string.char(1)))  
    cmd_sent=true  
  end  
  if (not armed or hold) and not was_hold then  
    PORT:writestring(pkt(string.char(2)))  
    cmd_sent=true  
  end  
  was_armed=armed  
  was_hold=(not armed or hold)  
  
  if auth_id then      
    local auth_fail_reason=nil    
    if vesc_st==4 then        
      -- confirmed FAILSAFE: block arming, this is a real fault        
      auth_fail_reason="VESC: FAILSAFE"        
    elseif not connected then        
      -- either never received a valid packet since boot, or went offline since        
      -- (offline watchdog clears `connected` on timeout) -- both are real faults        
      auth_fail_reason="VESC: no data"        
    end    
    -- 0 (IDLE), 1 (HOMING) and 3 (STOWED) are normal pre-arm/pre-homing states, not faults.    
    if auth_fail_reason ~= last_auth_fail_reason then    
      if auth_fail_reason then    
        arming:set_aux_auth_failed(auth_id, auth_fail_reason)    
      else    
        arming:set_aux_auth_passed(auth_id)    
      end    
      last_auth_fail_reason=auth_fail_reason    
    end    
  end
  
  qry_div=qry_div+1
  if qry_div>=40 then   -- query at 5 Hz (every 40 x 5ms cycles)  
    qry_div=0  
    if not cmd_sent then  
      PORT:writestring(pkt(string.char(4)))  
    end  
  end  
  
  local av=PORT:available()  
  if av:toint()>0 then  
    local chunk=PORT:readstring(av:toint())  
    if chunk then rx_buf=rx_buf..chunk end  
  end  
  
  while #rx_buf>=7 do  
    local s=rx_buf:find(string.char(2),1,true)  
    if not s then rx_buf="" break end  
    if s>1 then rx_buf=rx_buf:sub(s) end  
    if #rx_buf<7 then break end  
    local len=rx_buf:byte(2)  
    local pkt_len=len+5  
    if #rx_buf<pkt_len then break end  
    if rx_buf:byte(3)==0x24 and rx_buf:byte(4)==4 then  
      local payload=rx_buf:sub(3,2+len)  
      local rx_crc=(rx_buf:byte(3+len)<<8)|rx_buf:byte(3+len+1)  
      if crc16(payload)==rx_crc then    
        local new_st=rx_buf:byte(5)    
        if new_st~=vesc_st then        
		  logger:write("VESC","St,Old","hh",new_st,vesc_st)      
          gcs:send_text(6,"VESC: state -> "..state_name(new_st))        
          -- edge-based: homing (1) exiting to anything other than ACTIVE (2) = homing failed/aborted      
          -- this block only runs on the tick the state value actually changes, not on every packet      
          if vesc_st==1 and new_st~=2 and armed then          
            vehicle:set_mode(4)  -- force HOLD so the mode-edge Stow trigger fires on disarm    
            if arming:disarm() then          
              gcs:send_text(2,"VESC: homing failed -> DISARM")          
            end          
          end       
        end     
        vesc_st=new_st 
        connected=true      
        last_valid_pkt=now      
      end    
    end    
    rx_buf=rx_buf:sub(pkt_len+1)    
  end    
  if #rx_buf>256 then rx_buf="" end    
    
  -- level-based: failsafe, any time, any origin    
  if vesc_st==4 and armed then      
    vehicle:set_mode(4)  -- force HOLD so the mode-edge Stow trigger fires on disarm    
    if arming:disarm() then      
      gcs:send_text(2,"VESC: FAILSAFE -> DISARM")      
    end      
  end
  
  if vesc_st==2 and armed and not hold then    
    local pwm=SRV_Channels:get_output_pwm(26)    
    if pwm then    
      local s=(pwm-STEER_TRIM)/STEER_HALF_RANGE    
      local p=math.floor(s*1000+0.5)    
      if p>1000 then p=1000 elseif p<-1000 then p=-1000 end    
      PORT:writestring(pkt(string.char(3,(p>>8)&0xFF,p&0xFF)))    
    end    
  end    
  
  -- block throttle whenever VESC isn't confirmed ACTIVE, so a hung/failed  
  -- homing sequence (or dead VESC before the 3s offline watchdog fires)  
  -- can't leave CH3/SERVO3 throttle driven by AP_MotorsUGV unattended.  
  -- Gated on armed+not-hold: Hold already forces throttle=0 on its own  
  -- (ModeHold::update()), so no need to fight it there.  
  if THROTTLE_CHAN and armed and not hold and vesc_st~=2 then        
    SRV_Channels:set_output_pwm_chan_timeout(THROTTLE_CHAN, THROTTLE_SAFE_PWM, THROTTLE_OVERRIDE_MS)        
    if not throttle_override_active then  
      throttle_override_active=true  
      gcs:send_text(4,"VESC: throttle held -> "..state_name(vesc_st))  
    end  
  else  
    throttle_override_active=false  
  end
  
  -- health telemetry for companion computer (nano)  
  -- NOTE: send_named_int is not available on FW V4.7.0 (added later upstream);  
  -- send_named_float is used instead and works identically for this  
  -- purpose since vesc_st is a small integer (0-4) that round-trips  
  -- through a float with zero precision loss.  
  gcs:send_named_float('VESC_ST', vesc_st)
  
  if connected and now-last_valid_pkt>1000 and now-last_valid_pkt<=TIMEOUT then  
    gcs:send_text(4,"VESC: no data "..math.floor((now-last_valid_pkt)/100)/10 .."s")  
  end  
  
 if connected and now-last_valid_pkt>TIMEOUT then 
    connected=false    
    last_valid_pkt=now    
    if armed then    
      vehicle:set_mode(4)    
      if arming:disarm() then    
        gcs:send_text(2,"VESC: offline -> HOLD + DISARM")    
      else    
        gcs:send_text(3,"VESC: offline HOLD")    
      end    
    else    
      gcs:send_text(3,"VESC: offline")    
    end    
  end
  
  return update,POLL  
end  
  
PORT:begin(115200)  
PORT:set_flow_control(0)  
return update,1000