--[[
    Mustang GT350 CAN Integration Script
    VERSION: 3.0 (Final Integrated)
    LAST UPDATED: 2023-10-29 (Example Date)

    DESCRIPTION:
    This script provides comprehensive CAN bus integration for a custom ECU,
    replicating the necessary signals for the instrument cluster and other modules.

    FEATURES:
    - Full multi-rate CAN message scheduling based on vehicle specification.
    - "One-touch" latching starter control with security interlock and safety checks.
    - Dynamic gauge sweep on startup for RPM, Speed, Oil Temp, and Oil Pressure.
    - CAN-based exhaust cutout control with RPM/Pedal overrides and debounce timer.
    - Reactive drive mode implementation.
    - AC compressor anti-icing (freeze protection) logic.
    - Accurate, dynamic data for all essential gauges (Speedo, Tach, Temps, etc.).
--]]

------------------------------------------------------------------------------------------
-- CONSTANTS & GLOBAL STATE
------------------------------------------------------------------------------------------
-- Starter Control
local STARTER_PWM_CHANNEL = 0
local STARTER_MAX_RPM = 400
local STARTER_MIN_VOLTAGE = 6.0
local CRANKING_MAX_DURATION = 5.0

-- Exhaust Cutout Control
local CUTOUT_PWM_CHANNEL = 1
local CUTOUT_DEBOUNCE_DURATION = 2.0
local CUTOUT_RPM_THRESHOLD = 3000
local CUTOUT_PEDAL_THRESHOLD = 50.0

-- State Variables
local is_launch_control_requested = false -- Tracks the request from 0x3C8
local engine_state = "OFF" -- "OFF", "CRANKING", "RUNNING"
local cranking_timer = Timer.new()
local is_start_authorized = false
local is_ac_compressor_locked_out = false
local current_drive_mode = "NORMAL" -- NORMAL, SPORT, WEATHER, TRACK, DRAG
local exhaust_cutout_status = 5 -- From 0x439, default to 5 (Closed)
local current_cutout_state = "CLOSED" -- Actual state of our PWM output
local cutout_debounce_timer = Timer.new()
local gauge_sweep_state = "ACTIVE" -- "ACTIVE" or "NORMAL"
local gauge_sweep_timer = Timer.new()

------------------------------------------------------------------------------------------
-- POWERFUL BIT MANIPULATION HELPERS
------------------------------------------------------------------------------------------
local function pack_signal_motorola(buffer, start_bit, length, value)
    for i = 0, length - 1 do
        local bit_val = (value >> i) & 1
        local bit_pos = start_bit + i
        local byte_idx = math.floor(bit_pos / 8)
        local bit_in_byte = bit_pos % 8
        local motorola_bit_in_byte = 7 - bit_in_byte
        
        if bit_val == 1 then
            buffer[byte_idx + 1] = buffer[byte_idx + 1] | (1 << motorola_bit_in_byte)
        else
            buffer[byte_idx + 1] = buffer[byte_idx + 1] & (~(1 << motorola_bit_in_byte))
        end
    end
end

------------------------------------------------------------------------------------------
-- DATA BUFFERS (LATEST KNOWN-GOOD)
------------------------------------------------------------------------------------------
local data_buffers = {
    [0x47]  = {0x64, 0x48, 0x87, 0xe2, 0xF6, 0x38, 0x00, 0x00},
    [0x156] = {0x78, 0x7a, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00},
    [0x165] = {0x10, 0x40, 0x00, 0x00, 0x21, 0x43, 0x00, 0x00},
    [0x166] = {0x00, 0x00, 0x00, 0x00, 0xB3, 0x06, 0x00, 0x00},
    [0x167] = {0x72, 0x80, 0x0b, 0x00, 0x00, 0x19, 0xf1, 0x00},
    [0x171] = {0xf0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    [0x178] = {0x00, 0x00, 0x01, 0xff, 0x0e, 0x74, 0xda, 0x75},
    [0x179] = {0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x29},
    [0x200] = {0x00, 0x00, 0x7f, 0xff, 0x7f, 0xff, 0x00, 0x00},
    [0x202] = {0x00, 0xEF, 0x68, 0x00, 0x60, 0x00, 0x00, 0x00},
    [0x204] = {0xc0, 0x00, 0x7d, 0x01, 0x46, 0x00, 0x00, 0x00},
    [0x230] = {0xf0, 0x1c, 0x00, 0x00, 0x5a, 0x00, 0x00, 0x00},
    [0x421] = {0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x4a, 0x00},
    [0x424] = {0x00, 0x22, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00},
    [0x42d] = {0x00, 0x02, 0x89, 0x00, 0x00, 0x00, 0xf0, 0x00},
    [0x42f] = {0x7c, 0x0c, 0x00, 0x3e, 0x86, 0x32, 0x00, 0x00},
    [0x43d] = {0x00, 0x33, 0x35, 0xf9, 0x60, 0x00, 0x00, 0x00},
    [0x43e] = {0x00, 0x97, 0x10, 0x8d, 0x92, 0x2b, 0x99, 0x34},
    [0x447] = {0x22, 0x00, 0x00, 0x4B, 0x02, 0x00, 0x00, 0x00},
}
--0x178 reports launch on byte0 and tc
--0x242 algo con lc or tcs
--lc reported on 0x416 byte5
--check x43c
------------------------------------------------------------------------------------------
-- LOGIC & UPDATE FUNCTIONS
------------------------------------------------------------------------------------------
-- Add this new function to the LOGIC & UPDATE FUNCTIONS section
function update_LaunchControlAck_0x178()
    local data_buffer = data_buffers[0x178]
    
    -- We need to set or clear bit 6 (value 0x40) in the first byte (data_buffer[1])
    -- based on the request status, without disturbing other bits.

    if is_launch_control_requested then
        -- Set bit 6 by using a bitwise OR with 0x40
        data_buffer[1] = data_buffer[1] | 0x40
    else
        -- Clear bit 6 by using a bitwise AND with the inverse of 0x40
        data_buffer[1] = data_buffer[1] & (~0x40)
    end
end

function manage_gauge_sweep()
    -- This function only needs to do something while the sweep is active.
    if gauge_sweep_state == "ACTIVE" then
        local elapsed_time = gauge_sweep_timer:getElapsedSeconds()

        -- Control the exhaust cutout PWM based on elapsed time
        if elapsed_time < 0.6 then
            setPwmDuty(CUTOUT_PWM_CHANNEL, 0.10) -- Command OPEN
        elseif elapsed_time < 1.2 then
            setPwmDuty(CUTOUT_PWM_CHANNEL, 0.90) -- Command CLOSED
        elseif elapsed_time < 1.8 then
            setPwmDuty(CUTOUT_PWM_CHANNEL, 0.10) -- Command OPEN
        elseif elapsed_time < 2.4 then
            setPwmDuty(CUTOUT_PWM_CHANNEL, 0.90) -- Command CLOSED
        else
            -- The full 2-second sweep is over.
            gauge_sweep_state = "NORMAL"
            print("Gauge sweep complete. Resuming normal operation.")
            -- On the next cycle, the normal manage_exhaust_cutout() will take over.
        end
    end
end

function handle_starter_logic()
    local starter_request = getAuxDigital(1)
    local current_rpm = getSensor("RPM") or 0
    local battery_voltage = getSensor("BatteryVoltage") or 0
    local clutch_is_down = getOutput("clutchDownState")
    local safety_conditions_met = (clutch_is_down and battery_voltage > STARTER_MIN_VOLTAGE)

    if engine_state == "OFF" then
        if current_rpm >= STARTER_MAX_RPM then
            engine_state = "RUNNING"
        elseif starter_request and safety_conditions_met then
            engine_state = "CRANKING"
            cranking_timer:reset()
        end
    elseif engine_state == "CRANKING" then
        if is_start_authorized then
            setPwmDuty(STARTER_PWM_CHANNEL, 1.0)
        else
            setPwmDuty(STARTER_PWM_CHANNEL, 0.0)
        end

        local elapsed_crank_time = cranking_timer:getElapsedSeconds()
        if current_rpm >= STARTER_MAX_RPM then
            engine_state = "RUNNING"
            setPwmDuty(STARTER_PWM_CHANNEL, 0.0)
        elseif not safety_conditions_met then
            engine_state = "OFF"
            setPwmDuty(STARTER_PWM_CHANNEL, 0.0)
        elseif elapsed_crank_time >= CRANKING_MAX_DURATION then
            engine_state = "OFF"
            setPwmDuty(STARTER_PWM_CHANNEL, 0.0)
        end
    elseif engine_state == "RUNNING" then
        if current_rpm < STARTER_MAX_RPM then
            engine_state = "OFF"
        end
    end
end

-- Replace this function to add the guard clause
function manage_exhaust_cutout()
    -- IMPORTANT: Do not run normal logic if the startup sweep is active.
    if gauge_sweep_state == "ACTIVE" then
        return
    end

    -- === Part 1: Determine the Desired State (Normal operation logic) ===
    local car_system_wants_open = (exhaust_cutout_status == 2 or exhaust_cutout_status == 3)
    local rpm = getSensor("RPM") or 0
    local pedal = getSensor("AcceleratorPedal") or 0
    
    local desired_state
    if car_system_wants_open or (rpm > CUTOUT_RPM_THRESHOLD) or (pedal > CUTOUT_PEDAL_THRESHOLD) then
        desired_state = "OPEN"
    else
        desired_state = "CLOSED"
    end

    -- === Part 2: Apply Debounce Logic (Normal operation logic) ===
    if desired_state == current_cutout_state then
        cutout_debounce_timer:reset()
        return 
    end

    if cutout_debounce_timer:getElapsedSeconds() >= CUTOUT_DEBOUNCE_DURATION then
        print("Cutout state change confirmed. New state: " .. desired_state)
        if desired_state == "OPEN" then
            setPwmDuty(CUTOUT_PWM_CHANNEL, 0.10) -- 10% duty for OPEN
        else -- desired_state is "CLOSED"
        	setPwmDuty(CUTOUT_PWM_CHANNEL, 0.90) -- 90% duty for CLOSED
        end
        current_cutout_state = desired_state
    end
end

function update_Engine_Temps_0x156()
    local data_buffer = data_buffers[0x156]
    local clt = getSensor("Clt") or 0
    data_buffer[1] = math.floor(clt + 60) & 0xFF 

    if gauge_sweep_state == "ACTIVE" then
        data_buffer[2] = 247 -- 350F
    else
        data_buffer[2] = math.floor(clt + 70) & 0xFF
    end
end

function update_PowertrainData_3_0x43E()
    local data_buffer = data_buffers[0x43e]
    local oil_pressure_kpa

    -- Part 1: Fuel Pump Logic (remains the same)
    if getOutput("isFuelPumpOn") then
        data_buffer[2] = 0x90; data_buffer[5] = 0x72; data_buffer[7] = 0x4a
    else
        data_buffer[2] = 0x37; data_buffer[5] = 0x4b; data_buffer[7] = 0x48
    end

    -- Part 2: Oil Pressure Logic - with the test code removed
    if gauge_sweep_state == "ACTIVE" then
        -- Use the calibrated max value for the gauge sweep
        oil_pressure_kpa = 1000 
    else
        -- Normal operation: read the real sensor
        oil_pressure_kpa = getSensor("OilPressure") or 0
    end

    -- The conversion formula remains the same
    local raw_can_value = math.floor(oil_pressure_kpa * 1.015)
    local upper_4_bits = (raw_can_value >> 8) & 0x0F
    local lower_8_bits = raw_can_value & 0xFF
    data_buffer[8] = lower_8_bits
    data_buffer[7] = (data_buffer[7] & 0xF0) | upper_4_bits
end

function update_RPM_and_Pedal_0x204()
    local data_buffer = data_buffers[0x204]
    local rpm
    if gauge_sweep_state == "ACTIVE" then rpm = 9000 else rpm = getSensor("RPM") or 0 end
    local rpm_raw = math.floor((rpm / 2) + 0.5)
    data_buffer[4] = (rpm_raw >> 8) & 0xFF
    data_buffer[5] = rpm_raw & 0xFF

    local accel_pedal = getSensor("AcceleratorPedal") or 0
    local accel_raw = math.floor(accel_pedal * 10)
    local combined_value = 0xC000 | accel_raw 
    data_buffer[1] = (combined_value >> 8) & 0xFF
    data_buffer[2] = combined_value & 0xFF
end

function update_VehicleStatus_0x202()
    local data_buffer = data_buffers[0x202]
    local vehicle_speed_kmh
    local reverse_is_active = not getAuxDigital(0) 
    if reverse_is_active then data_buffer[1] = 0x08 else data_buffer[1] = 0x00 end

    if gauge_sweep_state == "ACTIVE" then vehicle_speed_kmh = 322 else vehicle_speed_kmh = getSensor("VehicleSpeed") or 0 end
    local raw_can_value = vehicle_speed_kmh * 99.1
    local capped_raw_value = math.min(raw_can_value, 31900)
    local final_value = math.floor(capped_raw_value)
    local high_byte = (final_value >> 8) & 0xFF
    local low_byte  = final_value & 0xFF
    data_buffer[7] = high_byte
    data_buffer[8] = low_byte
end

function update_Transmission_0x230()
    local data_buffer = data_buffers[0x230]
    local detected_gear = getOutput("detectedGear")
    local gear_nibble_value
    if detected_gear == 1 or detected_gear == 2 or detected_gear == 3 or detected_gear == 4 or detected_gear == 5 or detected_gear == 6 then
        gear_nibble_value = detected_gear
    else
        gear_nibble_value = 0x0F
    end
    local upper_nibble = gear_nibble_value << 4
    local lower_nibble = data_buffer[1] & 0x0F
    data_buffer[1] = upper_nibble | lower_nibble
    
    local reverse_is_active = not getAuxDigital(0) 
    if reverse_is_active then data_buffer[2] = 0x02 else data_buffer[2] = 0x1c end
end

function update_EngineAndClutch_0x167()
    local data_buffer = data_buffers[0x167]
    if engine_state == "CRANKING" then data_buffer[1] = 0x20 elseif engine_state == "RUNNING" then data_buffer[1] = 0x72 else data_buffer[1] = 0x00 end
    local clutch_is_full_down = getDigital(0)
    if clutch_is_full_down then data_buffer[3] = 0x00 else data_buffer[3] = 0x44 end
    local battery_voltage = getSensor("BatteryVoltage") or 0
    if battery_voltage >= 10.0 then data_buffer[7] = 0x68 else data_buffer[7] = 0xcb end
end

function update_BrakeStatus_0x165()
    local data_buffer = data_buffers[0x165]
    local brake_is_down = getDigital(2)
    if brake_is_down then data_buffer[1] = 0x20; data_buffer[5] = 0x10 else data_buffer[1] = 0x10; data_buffer[5] = 0x00 end
end

function update_data_0x47()
    local data_buffer = data_buffers[0x47]
    local battery_voltage = getSensor("BatteryVoltage") or 0
    if battery_voltage >= 10.0 then
        data_buffer[1]=0x20; data_buffer[2]=0x00; data_buffer[3]=0x00; data_buffer[4]=0x00; data_buffer[5]=0x00; data_buffer[6]=0x00; data_buffer[7]=0x00; data_buffer[8]=0x00
    else
        data_buffer[1]=0x04; data_buffer[2]=0xAD; data_buffer[3]=0x33; data_buffer[4]=0xF7; data_buffer[5]=0xE1; data_buffer[6]=0xFD; data_buffer[7]=0x00; data_buffer[8]=0x00
    end
end

function update_DriveMode_0x200()
    local data_buffer = data_buffers[0x200]
    if current_drive_mode == "SPORT" then data_buffer[7] = 0x10 elseif current_drive_mode == "WEATHER" then data_buffer[7] = 0x20 else data_buffer[7] = 0x00 end
end

function update_ExhaustState_0x424()
    local data_buffer = data_buffers[0x424]
    if exhaust_cutout_status == 2 or exhaust_cutout_status == 3 then data_buffer[3] = 0xA0 else data_buffer[3] = 0xC0 end
end

------------------------------------------------------------------------------------------
-- MULTI-RATE SCHEDULER
------------------------------------------------------------------------------------------
local messages_10ms   = { 0x167, 0x204, 0x43d, 0x43e }
local messages_20ms   = { 0x47, 0x165, 0x171, 0x200, 0x202, 0x230, 0x42f }
local messages_100ms  = { 0x156, 0x166, 0x178, 0x179, 0x421, 0x424, 0x42d }
local messages_1000ms = { 0x447 }

local timer_10ms = Timer.new();  local timer_20ms = Timer.new()
local timer_100ms = Timer.new(); local timer_1000ms = Timer.new()
timer_10ms:reset(); timer_20ms:reset(); timer_100ms:reset(); timer_1000ms:reset()

local function doSendTimedMessages()
    manage_gauge_sweep()

    if timer_10ms:getElapsedSeconds() >= 0.010 then
        timer_10ms:reset()
        update_PowertrainData_3_0x43E()
        update_RPM_and_Pedal_0x204()
        update_EngineAndClutch_0x167()
        for i=1, #messages_10ms do txCan(2, messages_10ms[i], 0, data_buffers[messages_10ms[i]]) end
    end

    if timer_20ms:getElapsedSeconds() >= 0.020 then
        timer_20ms:reset()
        update_data_0x47()
        update_Transmission_0x230()
        update_VehicleStatus_0x202()
        update_BrakeStatus_0x165()
        update_DriveMode_0x200()
        for i=1, #messages_20ms do txCan(2, messages_20ms[i], 0, data_buffers[messages_20ms[i]]) end
    end
    
    if timer_100ms:getElapsedSeconds() >= 0.100 then
        timer_100ms:reset()
        handle_starter_logic()
        manage_exhaust_cutout()
        update_Engine_Temps_0x156()
        update_ExhaustState_0x424()
		update_LaunchControlAck_0x178()
        for i=1, #messages_100ms do txCan(2, messages_100ms[i], 0, data_buffers[messages_100ms[i]]) end
    end

    if timer_1000ms:getElapsedSeconds() >= 1.000 then
        timer_1000ms:reset()
        for i=1, #messages_1000ms do txCan(2, messages_1000ms[i], 0, data_buffers[messages_1000ms[i]]) end
    end
end

------------------------------------------------------------------------------------------
-- CAN RECEIVE HANDLERS
------------------------------------------------------------------------------------------
function onAcReq(bus, id, dlc, data)
    local ac_status_byte = data[1]
    if ac_status_byte == 129 then setAcRequestState(1) else setAcRequestState(nil) end
    local high_byte = data[3]; local low_byte = data[4]
    local raw_temp_value = (high_byte * 256) + low_byte
    if is_ac_compressor_locked_out then
        if raw_temp_value >= 430 then
            print("AC Evap: Re-enabling compressor. Temp: " .. raw_temp_value)
            setAcDisabled(nil); is_ac_compressor_locked_out = false
        end
    else
        if raw_temp_value < 385 then
            print("AC Evap: Locking out compressor. Temp: " .. raw_temp_value)
            setAcDisabled(1); is_ac_compressor_locked_out = true
        end
    end
end

function onSecurityMessage_0x3B3(bus, id, dlc, data)
    local security_byte = data[8]
    if (security_byte & 2) == 0 then is_start_authorized = true else is_start_authorized = false end
end

-- Replace the old function with this new version that handles both tasks
function onDriveModeRequest_0x3C8(bus, id, dlc, data)
    -- Part 1: Drive Mode Logic (from before)
    local request_byte = data[3]
    if request_byte == 0 then current_drive_mode = "NORMAL"
    elseif request_byte == 1 then current_drive_mode = "SPORT"
    elseif request_byte == 2 then current_drive_mode = "WEATHER"
    elseif request_byte == 3 then current_drive_mode = "TRACK"
    elseif request_byte == 4 then current_drive_mode = "DRAG"
    end

    -- Part 2: Launch Control Request Logic (NEWLY ADDED)
    -- The request is in the last byte (byte 7, Lua index 8).
    -- The value 0x40 corresponds to bit 6 being set.
    local launch_control_byte = data[8]
    if (launch_control_byte & 0x40) ~= 0 then
        is_launch_control_requested = true
    else
        is_launch_control_requested = false
    end
end

function onExhaustStatus_0x439(bus, id, dlc, data)
    exhaust_cutout_status = data[3]
end

function canWheel(bus, id, dlc, data)
    if id == 0xC1 then print("Received message 0xC1") elseif id == 0xC5 then print("Received message 0xC5") end
end

------------------------------------------------------------------------------------------
-- MAIN TICK & INITIALIZATION
------------------------------------------------------------------------------------------
function onTick()
    doSendTimedMessages()
end

setTickRate(100)

startPwm(STARTER_PWM_CHANNEL, 1, 0.0)
startPwm(CUTOUT_PWM_CHANNEL, 100, 0.90)
cutout_debounce_timer:reset()
gauge_sweep_timer:reset()

canRxAdd(2, 0x326, onAcReq)
canRxAdd(2, 0x3B3, onSecurityMessage_0x3B3)
canRxAdd(2, 0x3C8, onDriveModeRequest_0x3C8)
canRxAdd(2, 0x439, onExhaustStatus_0x439)
canRxAdd(2, 0xC1, canWheel)
canRxAdd(2, 0xC5, canWheel)

print("Mustang GT350 CAN Script Initialized (v3.0).")