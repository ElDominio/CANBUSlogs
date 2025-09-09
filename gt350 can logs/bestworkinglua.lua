
--[[
    FINAL Merged & Corrected Lua CAN Script for 2016 Mustang GT350
    VERSION: Reverted Rx, Ready for Discussion
--]]

------------------------------------------------------------------------------------------
-- POWERFUL BIT MANIPULATION HELPERS (MOTOROLA / BIG-ENDIAN)
------------------------------------------------------------------------------------------
local function setBit(buffer, byte_idx, bit_idx, value)
    if value then
        buffer[byte_idx] = buffer[byte_idx] | (1 << bit_idx)
    else
        buffer[byte_idx] = buffer[byte_idx] & (~(1 << bit_idx))
    end
end

local function pack(buffer, start_bit, length, value)
    for i = 0, length - 1 do
        local bit_val = (value >> i) & 1
        local bit_pos = start_bit + i
        local byte_idx = math.floor(bit_pos / 8)
        local bit_in_byte = bit_pos % 8
        local motorola_bit_in_byte = 7 - bit_in_byte
        setBit(buffer, byte_idx + 1, motorola_bit_in_byte, bit_val == 1)
    end
end

local function unpack(buffer, start_bit, length)
    local value = 0
    for i = 0, length - 1 do
        local bit_pos = start_bit + i
        local byte_idx = math.floor(bit_pos / 8)
        local bit_in_byte = bit_pos % 8
        local motorola_bit_in_byte = 7 - bit_in_byte
        
        local extracted_bit = (buffer[byte_idx + 1] >> motorola_bit_in_byte) & 1
        
        if extracted_bit == 1 then
            value = value | (1 << i)
        end
    end
    return value
end

------------------------------------------------------------------------------------------
-- DATA BUFFERS (FROM YOUR WORKING SCRIPT)
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
    [0x204] = {0xc0, 0x00, 0x7d, 0x01, 0x46, 0x00, 0x00, 0x00},
    [0x230] = {0xf0, 0x1c, 0x00, 0x00, 0x5a, 0x00, 0x00, 0x00},
    [0x421] = {0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x4a, 0x00},
    [0x424] = {0x00, 0x22, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00},
    [0x42d] = {0x00, 0x02, 0x89, 0x00, 0x00, 0x00, 0xf0, 0x00},
    [0x42f] = {0x7c, 0x0c, 0x00, 0x3e, 0x86, 0x32, 0x00, 0x00},
    [0x439] = {0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00},
    [0x43d] = {0x00, 0x33, 0x35, 0xf9, 0x60, 0x00, 0x00, 0x00},
    [0x43e] = {0x00, 0x97, 0x10, 0x8d, 0x92, 0x2b, 0x99, 0x34},
    [0x447] = {0x22, 0x00, 0x00, 0x4B, 0x02, 0x00, 0x00, 0x00},
}

------------------------------------------------------------------------------------------
-- GLOBAL STATE & UPDATE FUNCTIONS
------------------------------------------------------------------------------------------
local counter0x47 = 0

function update_data_0x47(data_buffer)
    counter0x47 = counter0x47 + 1
    if counter0x47 == 5 then
        data_buffer[1] = 0x20; data_buffer[2] = 0x00; data_buffer[3] = 0x00
        data_buffer[4] = 0x00; data_buffer[5] = 0x00; data_buffer[6] = 0x00
    end
end

function update_EngineData_6_0x156(data_buffer)
    local clt = getSensor("Clt") or 0
    data_buffer[1] = math.floor(clt * 1.5 + 10) & 0xFF 
    data_buffer[2] = math.floor(clt * 1.5 + 20) & 0xFF
end

function update_EngVehicleSpThrottle_0x204(data_buffer)
    local rpm = getSensor("RPM") or 0
    local rpm_raw = math.floor((rpm / 2) + 0.5)
    data_buffer[4] = (rpm_raw >> 8) & 0xFF
    data_buffer[5] = rpm_raw & 0xFF
end

function update_PowertrainData_3_0x43E(data_buffer)
    if getOutput("isFuelPumpOn") then
        data_buffer[2] = 0x90; data_buffer[5] = 0x72; data_buffer[6] = 0x34
        data_buffer[7] = 0x4a; data_buffer[8] = 0x7b
    else
        data_buffer[2] = 0x37; data_buffer[5] = 0x4b; data_buffer[6] = 0x3f
        data_buffer[7] = 0x48; data_buffer[8] = 0x00
    end
end

------------------------------------------------------------------------------------------
-- MESSAGE CONFIGURATION
------------------------------------------------------------------------------------------
local message_update_functions = {
    [0x47]  = update_data_0x47,
    [0x156] = update_EngineData_6_0x156,
    [0x204] = update_EngVehicleSpThrottle_0x204,
    [0x43e] = update_PowertrainData_3_0x43E,
}

local configured_message_ids = {
    0x47, 0x156, 0x165, 0x166, 0x167, 0x171, 0x178, 0x179, 0x200, 0x204,
    0x230, 0x421, 0x424, 0x42d, 0x42f, 0x439, 0x43d, 0x43e, 0x447
}

local timer_10ms = Timer.new()
timer_10ms:reset()

------------------------------------------------------------------------------------------
-- SEND TIMED MESSAGES FUNCTION
------------------------------------------------------------------------------------------
local function doSendTimedMessages()
    if timer_10ms:getElapsedSeconds() >= 0.010 then
        timer_10ms:reset()
        
        for i = 1, #configured_message_ids do
            local id = configured_message_ids[i]
            local update_func = message_update_functions[id]
            local data_buffer = data_buffers[id]
            
            if update_func then
                update_func(data_buffer)
            end
            
            txCan(2, id, 0, data_buffer)
        end
    end
end

------------------------------------------------------------------------------------------
-- CAN RECEIVE HANDLERS
------------------------------------------------------------------------------------------
function onAcReq(bus, id, dlc, data)
    -- Based on your data capture:
    -- AC ON message has 0x95 in the second byte.
    -- AC OFF message has 0xA0 in the second byte.
    -- We will check the value of this byte to determine the A/C state.
    
    local ac_status_byte = data[1] -- Lua tables are 1-indexed

    if ac_status_byte == 129 then
        -- This matches the 'AC ON' data pattern.
	print(data[1])
     --   print("AC Request Detected from CAN: ON")
        setAcRequestState(1)
    else
        -- Any other value is assumed to be OFF.
     --   print("AC Request Detected from CAN: OFF")
        setAcRequestState(nil)
    end
end

function canWheel(bus, id, dlc, data)
	if id == 0xC1 then
		print("Received message 0xC1")
	elseif id == 0xC5 then
        print("Received message 0xC5")
	end
end

------------------------------------------------------------------------------------------
-- MAIN TICK FUNCTION
------------------------------------------------------------------------------------------
function onTick()
    doSendTimedMessages()
end

------------------------------------------------------------------------------------------
-- SCRIPT INITIALIZATION
------------------------------------------------------------------------------------------
setTickRate(100)

canRxAdd(2, 0x326, onAcReq) -- Reverted to original ID
canRxAdd(2, 0xC1, canWheel)
canRxAdd(2, 0xC5, canWheel)

print("Mustang GT350 CAN Script Initialized (Rx Reverted).")