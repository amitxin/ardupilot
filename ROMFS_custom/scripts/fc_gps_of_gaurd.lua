-- ForgeCopter GPS → Optical Flow Safety Guard
-- ArduCopter 4.6.3+ | Matek H743 + MicoAir MTF-01
-- 
-- Behaviour:
-- 1. Active after arming.
-- 2. When GPS fails and Optical Flow is working →
--    - switches EKF to SRC2 (Optical Flow)
--    - forces LOITER and locks the state
-- 3. When GPS fails and Optical Flow also fails →
--    - forces ALTHOLD (safe mode without position)
-- 4. Only a conscious mode change by the pilot unlocks the guard.
-- 5. After unlock a cooldown applies.
-- 6. Does not activate protection when already in LAND.
-- 7. Sends messages to OSD (Severity INFO/WARNING).
--
--  setup RCx_OPTION = 300 (Scripting1).  When this switch is pulled high, the source & mode (loiter/althold) will be automatically selected
--
-- Required parameters (your current settings are almost OK):
--   SCR_ENABLE = 1
--   EK3_SRC1_POSXY = 3   (GPS)          ← keep the mix
--   EK3_SRC1_VELXY = 5   (OpticalFlow)
--   EK3_SRC1_POSZ  = 1   (Baro)
--   EK3_SRC1_VELZ  = 3   (GPS)
--   EK3_SRC1_YAW   = 1   (Compass)
--   EK3_SRC2_POSXY = 0   (None)
--   EK3_SRC2_VELXY = 5   (OpticalFlow)
--   EK3_SRC2_POSZ  = 1   (Baro)
--   EK3_SRC2_VELZ  = 0
--   EK3_SRC2_YAW   = 1
--   EK3_SRC_OPTIONS = 0
--
-- Author: ForgeCopter
-- Version: 1.3 (2026-08-04) – GPS+OF fail → AltHold

---@diagnostic disable: need-check-nil
---@diagnostic disable: undefined-field

-- ============================================================
-- CONFIGURATION (edit here)
-- ============================================================
local EK3_SRC_GPS           = 0       -- EK3 gps source
local EK3_SRC_OFW           = 1       -- EK3 optical flow source
local UPDATE_RATE_MS        = 500     -- how often to check (ms)
local UPDATE_RATE_S         = 1       -- how often to check (s)
local GPS_BAD_DEBOUNCE_S    = 2.5     -- GPS must be bad for this many seconds before action
local GPS_MIN_SATS          = 6       -- below this number of satellites = bad
local GPS_MAX_HDOP          = 180     -- above this HDOP value = bad (×100)
local GPS_MAX_SPEED_ACC     = 0.7     -- m/s – above = bad (if available)
local GPS_MAX_HORIZ_ACC     = 2.5     -- m – above = bad (if available)
local OF_MIN_QUALITY        = 15      -- minimum Optical Flow quality (lenient)
local STATUS_MSG_INTERVAL_S = 12      -- how often to show status when locked
local UNLOCK_COOLDOWN_S     = 20      -- after pilot unlock – this many seconds of quiet
local STARTUP_DELAY_S       = 14      -- delay of startup message (after system settles)
local LOITER_MODE           = 5       -- LOITER mode number
local ALTHOLD_MODE          = 2       -- ALTHOLD mode number
local LAND_MODE             = 9       -- LAND mode number

-- Severity for OSD (DJI goggles like INFO and WARNING)
local SEV_INFO    = 6   -- MAV_SEVERITY_INFO
local SEV_WARNING = 4   -- MAV_SEVERITY_WARNING
local SEV_NOTICE  = 5   -- MAV_SEVERITY_NOTICE

-- ============================================================
-- INTERNAL STATE
-- ============================================================
local of_loit_lock_active   = false   -- true = we are in "OF Loiter locked" state
local gps_bad_timer         = 0       -- how long GPS has already been bad
local last_mode             = -1      -- previous mode
local last_status_msg_ms    = 0
local startup_done          = false   -- whether the startup message has already been sent
local script_boot_ms        = nil     -- moment of first script run
local unlock_cooldown_until = 0       -- timestamp (ms) until which cooldown after unlock applies
local sw_source_prev        = -1      -- prev source switch position
local sw_auto_switch_prev   = -1      -- previous auto source switch position
local auto_switch           = false   -- true when auto switching between sources is active

-- ============================================================
-- HELPER FUNCTIONS
-- ============================================================

local function send_osd(severity, text)
    gcs:send_text(severity, text)
end

local function is_gps_good()
    local inst = gps:primary_sensor()
    local status = gps:status(inst)

    -- No 3D fix = bad
    if status < gps.GPS_OK_FIX_3D then
        return false
    end

    -- Too few satellites
    local sats = gps:num_sats(inst)
    if sats ~= nil and sats < GPS_MIN_SATS then
        return false
    end

    -- HDOP too high
    local hdop = gps:get_hdop(inst)
    if hdop ~= nil and hdop > GPS_MAX_HDOP then
        return false
    end

    -- Speed accuracy (if available)
    local spd_acc = gps:speed_accuracy(inst)
    if spd_acc ~= nil and spd_acc > GPS_MAX_SPEED_ACC then
        return false
    end

    -- Horizontal accuracy (if available)
    local h_acc = gps:horizontal_accuracy(inst)
    if h_acc ~= nil and h_acc > GPS_MAX_HORIZ_ACC then
        return false
    end

    return true
end

local function is_optical_flow_usable()
    if not optical_flow then
        return false
    end
    if not optical_flow:enabled() then
        return false
    end
    if not optical_flow:healthy() then
        return false
    end
    local q = optical_flow:quality()
    if q ~= nil and q < OF_MIN_QUALITY then
        return false
    end
    return true
end

local function switch_to_source(src)
    if ahrs:get_posvelyaw_source_set() ~= src then
        ahrs:set_posvelyaw_source_set(src)  
    end
end 

local function force_loiter()
    local current = vehicle:get_mode()
    if current ~= LOITER_MODE then
        vehicle:set_mode(LOITER_MODE)
    end
end

-- ============================================================
-- MAIN LOOP
-- ============================================================
local function update()
    local now_ms = millis()

    -- Remember script start moment
    if script_boot_ms == nil then
        script_boot_ms = now_ms
        
        return update, UPDATE_RATE_MS
    end

    -- Startup message only after system has settled (EKF, OF, GPS etc.)
    if not startup_done then
        if (now_ms - script_boot_ms) >= (STARTUP_DELAY_S * 1000) then
            startup_done = true
            send_osd(SEV_INFO, "FC Guard: initialized & active")
        end
        return update, (UPDATE_RATE_S * 1000)
    end

    -- check switches are configured
    -- ek3 source RCx_FUNCTION = 90
    local rc_fn_source = rc:find_channel_for_option(90)

    -- auto loiter from RCx_FUNCTION = 300 (Scripting1)
    local rc_fn_auto_sw = rc:find_channel_for_option(300)
    
    -- Only when armed
    if not arming:is_armed() then

        if sw_source_prev ~= -1 then
            switch_to_source(sw_source_prev)
        end 

        -- Reset state after disarm
        if of_loit_lock_active then
            of_loit_lock_active = false
            gps_bad_timer = 0
            unlock_cooldown_until = 0

            last_mode = vehicle:get_mode()
            send_osd(SEV_INFO, "FC Guard: disarmed → unlocked")
        end

        return update, UPDATE_RATE_MS
    end

    local current_mode = vehicle:get_mode()
    if last_mode == -1 then
        last_mode = current_mode
    end

    local gps_ok = is_gps_good()
    local of_ok  = is_optical_flow_usable()

    if rc_fn_source then
        local sw_source = rc_fn_source:get_aux_switch_pos()
        if sw_source ~= sw_source_prev then    -- check for changes in source switch position
            sw_source_prev = sw_source         -- record new switch position so we can detect changes           

            send_osd(SEV_INFO, "FC Gaurd: EK3 SRC set to → " .. string.format("%d", sw_source_prev+1))
        else    
            send_osd(SEV_INFO, string.format("FC Gaurd: EK3 SRC → %d already set", sw_source_prev+1))
        end
    end

    -- if auto switch exists then read position from RCx_FUNCTION = 300 (Scripting1)
    if rc_fn_auto_sw then
        local sw_auto_loit = rc_fn_auto_sw:get_aux_switch_pos()
        if sw_auto_loit ~= sw_auto_switch_prev  then         -- check for changes in source auto switch position
            sw_auto_switch_prev = sw_auto_loit               -- record new switch position so we can detect changes
            
            if sw_auto_loit == 0 then                        -- pilot has pulled switch low
                auto_switch = false                          -- disable auto switching of source
                send_osd(SEV_INFO, "FC Guard: Auto switching disabled")
            elseif sw_auto_loit == 2 then                    -- pilot has pulled switch high
                auto_switch = true                           -- enable auto switching of source
                send_osd(SEV_INFO, "FC Gaurd: Auto switching enabled")
            end
        end
    end

    -- ========================================================
    -- LOCKED STATE (OF Loiter)
    -- ========================================================
    if of_loit_lock_active then
        -- Pilot (or system) changed mode → unlock
        if current_mode ~= LOITER_MODE then
            of_loit_lock_active = false
            gps_bad_timer = 0
            unlock_cooldown_until = now_ms + (UNLOCK_COOLDOWN_S * 1000)
            
            switch_to_source(sw_source_prev)
            send_osd(SEV_NOTICE, "FC Guard: pilot unlocked → SRC " .. string.format("%d", sw_source_prev+1))

            last_mode = current_mode
            return update, UPDATE_RATE_MS
        end

        -- If during lock Optical Flow also failed → switch to AltHold
        if not of_ok then
            of_loit_lock_active = false
            gps_bad_timer = 0
            unlock_cooldown_until = now_ms + (UNLOCK_COOLDOWN_S * 1000)

            switch_to_source(EK3_SRC_GPS)

            if current_mode ~= ALTHOLD_MODE and current_mode ~= LAND_MODE then
               vehicle:set_mode(ALTHOLD_MODE)
               send_osd(SEV_WARNING, "FC Guard: GPS+OF LOST → ALTHOLD")
            end

            last_mode = current_mode
            return update, UPDATE_RATE_MS
        end

        -- Keep OF source
        switch_to_source(EK3_SRC_OFW)
        
        -- Periodic status message
        if (now_ms - last_status_msg_ms) > (STATUS_MSG_INTERVAL_S * 1000) then
            last_status_msg_ms = now_ms
            if gps_ok then
                send_osd(SEV_WARNING, "FC Guard: OF ACTIVE → GPS back")
            else
                send_osd(SEV_WARNING, "FC Guard: OF ACTIVE → no GPS")
            end
        end

        last_mode = current_mode
        return update, UPDATE_RATE_MS
    end

    -- ========================================================
    -- NORMAL STATE – monitor GPS
    -- ========================================================

    -- Do not activate protection when already in LAND
    if current_mode == LAND_MODE then
        gps_bad_timer = 0
        last_mode = current_mode
        return update, UPDATE_RATE_MS
    end

    -- Cooldown after conscious unlock by pilot
    if now_ms < unlock_cooldown_until then
        last_mode = current_mode
        return update, UPDATE_RATE_MS
    end

    if not gps_ok then
        -- GPS bad – increase timer
        gps_bad_timer = gps_bad_timer + (UPDATE_RATE_MS / 1000.0)

        if gps_bad_timer >= GPS_BAD_DEBOUNCE_S then
            if of_ok then
                switch_to_source(EK3_SRC_OFW)

                if auto_switch then
                    -- GPS bad + OF good → classic Loiter protection
                    of_loit_lock_active = true
                                    
                    force_loiter()
                    send_osd(SEV_WARNING, "FC Guard: GPS LOST → OF LOITER")
                end
                last_status_msg_ms = now_ms
            else
                -- GPS bad + OF bad → force AltHold
                if auto_switch and current_mode ~= ALTHOLD_MODE and current_mode ~= LAND_MODE then
                    vehicle:set_mode(ALTHOLD_MODE)
                    
                    send_osd(SEV_WARNING, "FC Guard: GPS+OF LOST → ALTHOLD")
                    last_status_msg_ms = now_ms
                elseif (now_ms - last_status_msg_ms) > 5000 then
                    send_osd(SEV_WARNING, "FC Guard: GPS+OF LOST → ALTHOLD")
                    last_status_msg_ms = now_ms
                end
            end
        end
    else
        -- GPS good – reset timer
        gps_bad_timer = 0
    end

    last_mode = current_mode
    return update, UPDATE_RATE_MS
end

-- Start
return update()

