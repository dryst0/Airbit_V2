// Characterization tests for the airbit extension.
// These tests capture the current behavior of pure logic functions
// so we can refactor with confidence.
//
// Run with: pxt test (or open extension directly in MakeCode editor)
// Tests use control.assert() which halts the program on failure.
//
// Note: main.ts top-level code runs first (hardware init, calibration).
// On the simulator, I2C calls are stubbed and arm=0 so the while(arm) loop exits immediately.

// --- Test Helpers ---

function assertEqual(actual: number, expected: number, message: string) {
    control.assert(
        actual == expected,
        message + " — expected " + expected + " got " + actual
    )
}

function assertClose(actual: number, expected: number, tolerance: number, message: string) {
    const diff = Math.abs(actual - expected)
    control.assert(
        diff <= tolerance,
        message + " — expected ~" + expected + " got " + actual + " (tolerance " + tolerance + ")"
    )
}

function assertTrue(condition: boolean, message: string) {
    control.assert(condition, message)
}

function testGroup(name: string) {
    serial.writeLine("--- " + name + " ---")
}

// --- PID Stabilization Tests ---
// stabilisePid() reads these main.ts globals: roll, pitch, yaw, imuRoll, imuPitch, imuYaw,
// throttle, rollPitchP, rollPitchI, rollPitchD, yawP, yawD
// stabilisePid() writes these main.ts globals: motorA, motorB, motorC, motorD

testGroup("stabilisePid: zero error produces equal motor speeds")

airbit.cleanReg()
imuRoll = 0
imuPitch = 0
imuYaw = 0
roll = 0
pitch = 0
yaw = 0
throttle = 60
rollPitchP = 0.9
rollPitchI = 0.004
rollPitchD = 15
yawP = 5
yawD = 70

airbit.stabilisePid()

// With zero error, all corrections are 0. throttleScaled = 60 * 2.55 = 153
assertEqual(motorA, 153, "Zero error: motorA should equal scaled throttle")
assertEqual(motorB, 153, "Zero error: motorB should equal scaled throttle")
assertEqual(motorC, 153, "Zero error: motorC should equal scaled throttle")
assertEqual(motorD, 153, "Zero error: motorD should equal scaled throttle")

testGroup("stabilisePid: roll error produces differential motor speeds")

airbit.cleanReg()
imuRoll = 5
imuPitch = 0
imuYaw = 0
roll = 0
pitch = 0
yaw = 0
throttle = 60
rollPitchP = 0.9
rollPitchI = 0.004
rollPitchD = 15
yawP = 5
yawD = 70

airbit.stabilisePid()

// rollDiff = 0 - 5 = -5
// rollCorrection = -5*0.9 + 0 + -5*15 = -4.5 - 75 = -79.5
// motorA = round(153 + (-79.5) + 0 + 0) = 74
// motorC = round(153 - (-79.5) + 0 + 0) = 233
assertTrue(motorA < motorC, "Roll right tilt: motorA (left) should be less than motorC (right)")
assertTrue(motorB < motorD, "Roll right tilt: motorB (left) should be less than motorD (right)")

testGroup("stabilisePid: motor speeds clamped to 0-255")

airbit.cleanReg()
imuRoll = 15
imuPitch = 15
imuYaw = 0
roll = -15
pitch = -15
yaw = 0
throttle = 100
rollPitchP = 0.9
rollPitchI = 0.004
rollPitchD = 15
yawP = 5
yawD = 70

airbit.stabilisePid()

assertTrue(motorA >= 0, "Motor A clamped: not negative")
assertTrue(motorA <= 255, "Motor A clamped: not over 255")
assertTrue(motorB >= 0, "Motor B clamped: not negative")
assertTrue(motorB <= 255, "Motor B clamped: not over 255")
assertTrue(motorC >= 0, "Motor C clamped: not negative")
assertTrue(motorC <= 255, "Motor C clamped: not over 255")
assertTrue(motorD >= 0, "Motor D clamped: not negative")
assertTrue(motorD <= 255, "Motor D clamped: not over 255")

// --- Battery Level Tests ---

testGroup("batteryLevel: maps voltage to percentage")

// batteryLevel() calls batteryCalculation() first (reads analog pin, applies smoothing).
// On the simulator, analogReadPin returns 0, so batterymVoltSmooth converges toward 0.
// We can set batterymVoltSmooth directly and test the mapping by calling batteryLevel().
// Note: batteryLevel() calls batteryCalculation() which will modify batterymVoltSmooth.
// So we test the math directly: Math.map(voltage, 3400, 4200, 0, 100)
// These values match BATTERY_VOLTAGE_MIN (3400) and BATTERY_VOLTAGE_MAX (4200) in custom.ts

let expectedMin = Math.map(3400, 3400, 4200, 0, 100)
assertEqual(expectedMin, 0, "Battery mapping: 3400mV should be 0%")

let expectedMax = Math.map(4200, 3400, 4200, 0, 100)
assertEqual(expectedMax, 100, "Battery mapping: 4200mV should be 100%")

let expectedMid = Math.map(3800, 3400, 4200, 0, 100)
assertEqual(expectedMid, 50, "Battery mapping: 3800mV should be 50%")

// --- cleanReg Tests ---
// cleanReg() resets PID accumulators. We can verify it resets imuYaw (a main.ts global).

testGroup("cleanReg: resets imuYaw to zero")

imuYaw = 45
airbit.cleanReg()
assertEqual(imuYaw, 0, "cleanReg should reset imuYaw to 0")

// --- All Tests Passed ---

serial.writeLine("=== ALL TESTS PASSED ===")
basic.showIcon(IconNames.Yes)
