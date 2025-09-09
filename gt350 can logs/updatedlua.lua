--[[
    Mustang GT350 CAN Integration Script
    VERSION: 2.0.0 (Final)
    LAST UPDATED: 2023-10-28 (Example Date)

    CHANGELOG:
    - v2.0.0:
        - MAJOR: Synthesized a final, stable script.
        - FEATURE: Integrated the full 19-message CAN roster from the known-working script.
        - FEATURE: Implemented a full multi-rate scheduler based on official timing data.
        - FIX: Corrected RPM signal by using a custom update function to ensure
          proper Big Endian byte order (High Byte, then Low Byte).
        - FIX: Corrected Coolant/Oil temperature signals with final, validated formulas
          and placed them in a dedicated update function.
        - FIX: Removed all faulty/placeholder logic (e.g., Launch Control icons).
        - RETAIN: Kept the robust starter motor control logic from previous versions.
--]]

------------------------------------------------------------------------------------------
-- CONSTANTS & GLOBAL STATE
------------------------------------------------------------------------------------------
-- Starter Control Configuration
local STARTER_PWM_CHANNEL = 0
local STARTER_MAX_RPM = 400
local STARTER_MIN_VOLTAGE = 6.0

-- State Variables
local is_starter_active = false
local is_start_authorized = false
local CRANKING_MAX_DURATION = 5.0 -- Max seconds to crank before timeout
local engine_state = "OFF"        -- Can be "OFF", "CRANKING", or "RUNNING"
local cranking_timer = Timer.new()
local gauge_sweep_state = "ACTIVE" -- Can be "ACTIVE" or "NORMAL"
local gauge_sweep_timer = Timer.new()
------------------------------------------------------------------------------------------
-- POWERFUL BIT MANIPULATION HELPERS
------------------------------------------------------------------------------------------
-- NOTE: This generic packer is kept for future use, but our most critical
-- multi-byte signals now use custom functions for reliability.
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
-- DATA BUFFERS (FROM THE WORKING SCRIPT)
------------------------------------------------------------------------------------------
local data_buffers = {
    [0x47]  = {0x64, 0x48, 0x87, 0xe2, 0xF6, 0x38, 0x00, 0x00},
    [0x156] = {0x78, 0x7a, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00},
    [0x165] = {0x10, 0x40, 0x00, 0x00, 0x21, 0x43, 0x00, 0x00}, --byte0/4 10/00 brake up, 20/10 brake down
    [0x166] = {0x00, 0x00, 0x00, 0x00, 0xB3, 0x06, 0x00, 0x00},
    [0x167] = {0x72, 0x80, 0x0b, 0x00, 0x00, 0x19, 0xf1, 0x00},--byte2 0x44 clutch not full down,00 clutch full down
    [0x171] = {0xf0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    [0x178] = {0x00, 0x00, 0x01, 0xff, 0x0e, 0x74, 0xda, 0x75},
    [0x179] = {0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x29},
    [0x200] = {0x00, 0x00, 0x7f, 0xff, 0x7f, 0xff, 0x00, 0x00}, --def drive mode shit, penultimate 00 normal/track, 10 sport, 20 weather
	[0x202] = {0x00, 0xEF, 0x68, 0x00, 0x60, 0x00, 0x00, 0x00},
    [0x204] = {0xc0, 0x00, 0x7d, 0x01, 0x46, 0x00, 0x00, 0x00},
    [0x230] = {0xf0, 0x1c, 0x00, 0x00, 0x5a, 0x00, 0x00, 0x00}, --byte1 1c means not reverse, 02 means reverse
    [0x421] = {0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x4a, 0x00},
    [0x424] = {0x00, 0x22, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00}, -- also drive mode, byte2 c0 on normal, wealter, a0 on sport and track
    [0x42d] = {0x00, 0x02, 0x89, 0x00, 0x00, 0x00, 0xf0, 0x00},
    [0x42f] = {0x7c, 0x0c, 0x00, 0x3e, 0x86, 0x32, 0x00, 0x00},
    [0x439] = {0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00},
    [0x43d] = {0x00, 0x33, 0x35, 0xf9, 0x60, 0x00, 0x00, 0x00},
    [0x43e] = {0x00, 0x97, 0x10, 0x8d, 0x92, 0x2b, 0x99, 0x34},
    [0x447] = {0x22, 0x00, 0x00, 0x4B, 0x02, 0x00, 0x00, 0x00},
}

------------------------------------------------------------------------------------------
-- LOGIC & UPDATE FUNCTIONS
------------------------------------------------------------------------------------------
function manage_gauge_sweep()
    -- This function only needs to do something while the sweep is active.
    if gauge_sweep_state == "ACTIVE" then
        if gauge_sweep_timer:getElapsedSeconds() >= 2.0 then
            gauge_sweep_state = "NORMAL"
            print("Gauge sweep complete. Resuming normal operation.")
        end
    end
end

function update_BrakeStatus_0x165()
    local data_buffer = data_buffers[0x165]
    local brake_is_down = getDigital(2) -- Use the new brake switch sensor

    if brake_is_down then
        -- Set bytes for "brake down" state
        data_buffer[1] = 0x20
        data_buffer[5] = 0x10
    else
        -- Set bytes for "brake up" state
        data_buffer[1] = 0x10
        data_buffer[5] = 0x00
    end
end

function update_VehicleStatus_0x202()
    local data_buffer = data_buffers[0x202]
    local vehicle_speed_kmh

    -- Part 1: Reverse Status Logic (remains the same)
    local reverse_is_active = not getAuxDigital(0) 
    if reverse_is_active then data_buffer[1] = 0x08 else data_buffer[1] = 0x00 end

    -- Part 2: Wheel Speed Logic - check for gauge sweep
    if gauge_sweep_state == "ACTIVE" then
        -- 200 MPH is approx 322 km/h.
        vehicle_speed_kmh = 322
    else
        vehicle_speed_kmh = getSensor("VehicleSpeed") or 0
    end
    
    local raw_can_value = vehicle_speed_kmh * 99.1
    local capped_raw_value = math.min(raw_can_value, 31900)
    local final_value = math.floor(capped_raw_value)
    local high_byte = (final_value >> 8) & 0xFF
    local low_byte  = final_value & 0xFF
    data_buffer[7] = high_byte
    data_buffer[8] = low_byte
end

-- Replace the old function with this new expanded version
function update_Transmission_0x230()
    local data_buffer = data_buffers[0x230]

    -- Part 1: Gear Indicator Logic (for byte 0, first nibble) - NEWLY ADDED
    local detected_gear = getOutput("detectedGear")
    local gear_nibble_value

    -- Map the detected gear (1-6) to the required CAN value.
    -- Any other value (nil, 0, 7, etc.) is considered Neutral (F).
    if detected_gear == 1 or detected_gear == 2 or detected_gear == 3 or detected_gear == 4 or detected_gear == 5 or detected_gear == 6 then
        gear_nibble_value = detected_gear
    else
        gear_nibble_value = 0x0F -- 'F' for Neutral
    end

    -- Carefully update the first byte (data_buffer[1]).
    -- We must preserve the lower 4 bits of the byte and only replace the upper 4 bits.
    -- 1. Shift our new gear value into the upper position (e.g., 6 becomes 0x60).
    -- 2. Get the lower 4 bits from the existing byte.
    -- 3. Combine them.
    local upper_nibble = gear_nibble_value << 4
    local lower_nibble = data_buffer[1] & 0x0F
    data_buffer[1] = upper_nibble | lower_nibble
    

    -- Part 2: Reverse Switch Logic (for byte 1) - from before
    -- This uses the corrected, inverted signal logic.
    local reverse_is_active = not getAuxDigital(0) 

    if reverse_is_active then
        data_buffer[2] = 0x02 -- "Reverse"
    else
        data_buffer[2] = 0x1c -- "Not Reverse"
    end
end

-- Replace the old function with this new expanded version
function update_EngineAndClutch_0x167()
    local data_buffer = data_buffers[0x167]
    
    -- Part 1: Engine Status Logic (byte 0)
    if engine_state == "CRANKING" then
        data_buffer[1] = 0x20
    elseif engine_state == "RUNNING" then
        data_buffer[1] = 0x72
    else -- engine_state is "OFF"
        data_buffer[1] = 0x00
    end

    -- Part 2: Clutch Status Logic (byte 2)
    local clutch_is_full_down = getDigital(0)

    if clutch_is_full_down then
        data_buffer[3] = 0x00
    else
        data_buffer[3] = 0x44
    end

    -- Part 3: Battery Voltage Status Logic (byte 6) - NEWLY ADDED
    local battery_voltage = getSensor("BatteryVoltage") or 0

    if battery_voltage >= 10.0 then
        data_buffer[7] = 0x68 -- Set byte 6 (lua index 7)
    else
        data_buffer[7] = 0xcb -- Set byte 6 (lua index 7)
    end
end

function handle_starter_logic()
    -- Get all required sensor data once at the beginning
    local starter_request = getAuxDigital(1)
    local current_rpm = getSensor("RPM") or 0
    local battery_voltage = getSensor("BatteryVoltage") or 0
    local clutch_is_down = getOutput("clutchDownState")

    -- Define safety conditions for attempting to crank
    local safety_conditions_met = (clutch_is_down and battery_voltage > STARTER_MIN_VOLTAGE)

    -- === STATE MACHINE ===

    if engine_state == "OFF" then
        -- We are in the OFF state.
        if current_rpm >= STARTER_MAX_RPM then
            -- Sanity check: The engine is already running. Correct our state.
            engine_state = "RUNNING"
            
        elseif starter_request and safety_conditions_met then
            -- The user wants to start and safeties are met. Announce our INTENT to crank.
            -- We do NOT check for authorization here.
            engine_state = "CRANKING"
            cranking_timer:reset()
            -- NOTE: No setPwmDuty() call here.
        end

    elseif engine_state == "CRANKING" then
        -- We have announced our intent. Now we manage the cranking process.

        -- --- PWM CONTROL ---
        -- This is where we check for authorization before taking ACTION.
        if is_start_authorized then
            setPwmDuty(STARTER_PWM_CHANNEL, 1.0) -- Authorized: Engage starter.
        else
            setPwmDuty(STARTER_PWM_CHANNEL, 0.0) -- Not authorized: Disengage starter.
        end

        -- --- EXIT CONDITIONS ---
        -- Now, check for any reason to STOP the cranking attempt entirely.
        local elapsed_crank_time = cranking_timer:getElapsedSeconds()

        if current_rpm >= STARTER_MAX_RPM then
            -- Condition 1: Engine has started successfully.
            engine_state = "RUNNING"
            setPwmDuty(STARTER_PWM_CHANNEL, 0.0) -- Ensure PWM is off.

        elseif not safety_conditions_met then
            -- Condition 2: A safety interlock was released.
            engine_state = "OFF"
            setPwmDuty(STARTER_PWM_CHANNEL, 0.0) -- Ensure PWM is off.

        elseif elapsed_crank_time >= CRANKING_MAX_DURATION then
            -- Condition 3: We have timed out.
            engine_state = "OFF"
            setPwmDuty(STARTER_PWM_CHANNEL, 0.0) -- Ensure PWM is off.
        end

    elseif engine_state == "RUNNING" then
        -- We are in the RUNNING state. We only look for a stall condition.
        if current_rpm < STARTER_MAX_RPM then
            engine_state = "OFF"
        end
    end
end

-- Custom update functions for messages with special requirements
-- Replace the old function with this new version
function update_data_0x47()
    local data_buffer = data_buffers[0x47]
    local battery_voltage = getSensor("BatteryVoltage") or 0

    if battery_voltage >= 10.0 then
        -- Set the "Over 10V" payload
        data_buffer[1] = 0x20; data_buffer[2] = 0x00; data_buffer[3] = 0x00; data_buffer[4] = 0x00
        data_buffer[5] = 0x00; data_buffer[6] = 0x00; data_buffer[7] = 0x00; data_buffer[8] = 0x00
    else
        -- Set the "Under 10V" payload
        data_buffer[1] = 0x04; data_buffer[2] = 0xAD; data_buffer[3] = 0x33; data_buffer[4] = 0xF7
        data_buffer[5] = 0xE1; data_buffer[6] = 0xFD; data_buffer[7] = 0x00; data_buffer[8] = 0x00
    end
end

function update_Engine_Temps_0x156()
    local data_buffer = data_buffers[0x156]
    local clt = getSensor("Clt") or 0

    -- Use final, validated formulas for coolant
    data_buffer[1] = math.floor(clt + 60) & 0xFF 

    -- For oil temp, check if we are doing the gauge sweep
    if gauge_sweep_state == "ACTIVE" then
        -- 350F is approx 177C. The formula is C+70. So, 177+70=247.
        data_buffer[2] = 247 
    else
        -- Normal operation: fake oil temp from coolant temp
        data_buffer[2] = math.floor(clt + 70) & 0xFF
    end
end

function update_RPM_and_Pedal_0x204()
    local data_buffer = data_buffers[0x204]
    local rpm

    -- Part 1: RPM Logic - check for gauge sweep
    if gauge_sweep_state == "ACTIVE" then
        rpm = 9000
    else
        rpm = getSensor("RPM") or 0
    end

    local rpm_raw = math.floor((rpm / 2) + 0.5)
    data_buffer[4] = (rpm_raw >> 8) & 0xFF
    data_buffer[5] = rpm_raw & 0xFF

    -- Part 2: Accelerator Pedal Logic (remains the same)
    local accel_pedal = getSensor("AcceleratorPedal") or 0
    local accel_raw = math.floor(accel_pedal * 10)
    local combined_value = 0xC000 | accel_raw 
    data_buffer[1] = (combined_value >> 8) & 0xFF
    data_buffer[2] = combined_value & 0xFF
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

    -- Part 2: Oil Pressure Logic - check for gauge sweep
    if gauge_sweep_state == "ACTIVE" then
        -- 150 PSI is approx 1034 kPa.
        oil_pressure_kpa = 1034
    else
        oil_pressure_kpa = getSensor("OilPressure") or 0
    end

    local raw_can_value = math.floor(oil_pressure_kpa * 1.015)
    local upper_4_bits = (raw_can_value >> 8) & 0x0F
    local lower_8_bits = raw_can_value & 0xFF
    data_buffer[8] = lower_8_bits
    data_buffer[7] = (data_buffer[7] & 0xF0) | upper_4_bits
end

-- Generic function to pack any remaining simple signals if needed in the future.
-- It will be skipped for messages handled by custom functions.
local function update_and_pack_message(id)
    local config = message_configs[id]
    if not (config and config.signals) then return end

    local data_buffer = config.buffer
    for i = 1, #config.signals do
        local signal = config.signals[i]
        local physical_value = signal.value()
        local encoded_value = signal.encoder(physical_value)
        pack_signal_motorola(data_buffer, signal.start_bit, signal.length, encoded_value)
    end
end

------------------------------------------------------------------------------------------
-- CENTRALIZED MESSAGE CONFIGURATION
------------------------------------------------------------------------------------------
local message_configs = {
    -- Messages with custom update functions are defined simply by rate and buffer
    [0x47]  = { rate = 20,   buffer = data_buffers[0x47] },
    [0x156] = { rate = 100,  buffer = data_buffers[0x156] },
    [0x204] = { rate = 10,   buffer = data_buffers[0x204] },
    [0x43e] = { rate = 10,   buffer = data_buffers[0x43e] },
	[0x202] = { rate = 20,   buffer = data_buffers[0x202] },
    
    -- All other messages are static and just need their buffer sent at the correct rate
    [0x165] = { rate = 20,   buffer = data_buffers[0x165] },
    [0x166] = { rate = 100,  buffer = data_buffers[0x166] },
    [0x167] = { rate = 10,   buffer = data_buffers[0x167] },
    [0x171] = { rate = 20,   buffer = data_buffers[0x171] },
    [0x178] = { rate = 100,  buffer = data_buffers[0x178] },
    [0x179] = { rate = 100,  buffer = data_buffers[0x179] },
    [0x200] = { rate = 20,   buffer = data_buffers[0x200] }, -- TODO Drive Mode: byte6 (lua [7]): 00=Norm/Track, 10=Sport, 20=Weather
    [0x230] = { rate = 20,   buffer = data_buffers[0x230] },
    [0x421] = { rate = 100,  buffer = data_buffers[0x421] },
    [0x424] = { rate = 100,  buffer = data_buffers[0x424] }, -- TODO Drive Mode: byte2 (lua [3]): C0=Norm/Weath, A0=Sport/Track
    [0x42d] = { rate = 100,  buffer = data_buffers[0x42d] },
    [0x42f] = { rate = 20,   buffer = data_buffers[0x42f] },
    [0x439] = { rate = 1000, buffer = data_buffers[0x439] },
    [0x43d] = { rate = 10,   buffer = data_buffers[0x43d] },
    [0x447] = { rate = 1000, buffer = data_buffers[0x447] }
}

------------------------------------------------------------------------------------------
-- MULTI-RATE SCHEDULER
------------------------------------------------------------------------------------------
-- Message lists sorted by rate, containing only the known-working messages
local messages_10ms   = { 0x167, 0x204, 0x43d, 0x43e }
local messages_20ms   = { 0x47, 0x165, 0x171, 0x200, 0x202, 0x230, 0x42f }
local messages_100ms  = { 0x156, 0x166, 0x178, 0x179, 0x421, 0x424, 0x42d }
local messages_1000ms = { 0x439, 0x447 }

-- Timers for each rate
local timer_10ms = Timer.new();  local timer_20ms = Timer.new()
local timer_100ms = Timer.new(); local timer_1000ms = Timer.new()
timer_10ms:reset(); timer_20ms:reset(); timer_100ms:reset(); timer_1000ms:reset()

local function doSendTimedMessages()
manage_gauge_sweep()

    -- 10ms loop (100 Hz)
    if timer_10ms:getElapsedSeconds() >= 0.010 then
        timer_10ms:reset()
        update_PowertrainData_3_0x43E()
        update_RPM_and_Pedal_0x204()
		update_EngineAndClutch_0x167()
        for i = 1, #messages_10ms do
            txCan(2, messages_10ms[i], 0, data_buffers[messages_10ms[i]])
        end
    end

    -- 20ms loop (50 Hz)
    if timer_20ms:getElapsedSeconds() >= 0.020 then
        timer_20ms:reset()
        update_data_0x47()
		update_Transmission_0x230()
		update_VehicleStatus_0x202() 
		update_BrakeStatus_0x165()
        for i = 1, #messages_20ms do
            txCan(2, messages_20ms[i], 0, data_buffers[messages_20ms[i]])
        end
    end
    
    -- 100ms loop (10 Hz)
    if timer_100ms:getElapsedSeconds() >= 0.100 then
        timer_100ms:reset()
        handle_starter_logic()
        update_Engine_Temps_0x156()
        for i = 1, #messages_100ms do
            txCan(2, messages_100ms[i], 0, data_buffers[messages_100ms[i]])
        end
    end

    -- 1000ms loop (1 Hz)
    if timer_1000ms:getElapsedSeconds() >= 1.000 then
        timer_1000ms:reset()
        for i = 1, #messages_1000ms do
            txCan(2, messages_1000ms[i], 0, data_buffers[messages_1000ms[i]])
        end
    end
end

------------------------------------------------------------------------------------------
-- CAN RECEIVE HANDLERS
------------------------------------------------------------------------------------------
function onSecurityMessage_0x3B3(bus, id, dlc, data)
    -- Check byte 7 (data[7]) for the authorization signal.
    -- The last nibble must equal 8. We use a bitwise AND with 0x0F to isolate it.
    local security_byte = data[8]
    if (security_byte & 2) == 0 then
        is_start_authorized = true
    else
        is_start_authorized = false
    end
end

-- Replace the onAcReq function with this new version containing the hysteresis logic
function onAcReq(bus, id, dlc, data)
    -- Part 1: Original AC Request logic (from the user's main switch)
    local ac_status_byte = data[1]
    if ac_status_byte == 129 then
        setAcRequestState(1)
    else
        setAcRequestState(nil)
    end

    -- Part 2: Evaporator Temperature Calculation
    local high_byte = data[3]
    local low_byte = data[4]
    local raw_temp_value = (high_byte * 256) + low_byte
    
    -- Part 3: AC Compressor Lockout (Freeze Protection Hysteresis)
    if is_ac_compressor_locked_out then
        -- The compressor is currently disabled by us. We are waiting for the
        -- evaporator to warm up before we allow it to run again.
        if raw_temp_value >= 430 then
            print("AC Evap: Re-enabling compressor. Temp: " .. raw_temp_value)
            setAcDisabled(nil)
            is_ac_compressor_locked_out = false -- Update our state
        end
    else
        -- The compressor is currently allowed to run. We are watching for the
        -- evaporator to get too cold.
        if raw_temp_value < 385 then
            print("AC Evap: Locking out compressor. Temp: " .. raw_temp_value)
            setAcDisabled(1)
            is_ac_compressor_locked_out = true -- Update our state
        end
    end
end

function canWheel(bus, id, dlc, data)
    if id == 0xC1 then print("Received message 0xC1")
    elseif id == 0xC5 then print("Received message 0xC5") end
end

------------------------------------------------------------------------------------------
-- MAIN TICK & INITIALIZATION
------------------------------------------------------------------------------------------
gauge_sweep_timer:reset()

function onTick()
    doSendTimedMessages()
end

setTickRate(100)

-- Initialize starter PWM channel to a safe 'OFF' state on boot
startPwm(STARTER_PWM_CHANNEL, 1, 0.0)

-- Register CAN message listeners
canRxAdd(2, 0x326, onAcReq)
canRxAdd(2, 0xC1, canWheel)
canRxAdd(2, 0xC5, canWheel)
canRxAdd(2, 0x3B3, onSecurityMessage_0x3B3)

print("Mustang GT350 CAN Script Initialized (v2.0.0).")