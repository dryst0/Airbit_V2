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

// --- Drone Stabilization Tests ---
// The stabilize() function reads the pilot's desired angles (roll, pitch, yaw),
// compares them to the actual measured angles, and calculates motor speeds.

testGroup("when the drone is level, all motors spin at the same speed")

airbit.resetPidState()
measuredRoll = 0
measuredPitch = 0
measuredYaw = 0
roll = 0
pitch = 0
yaw = 0
throttle = 60
rollPitchP = 0.9
rollPitchI = 0.004
rollPitchD = 15
yawP = 5
yawD = 70

airbit.stabilize()

// With zero error, all corrections are 0. throttleScaled = 60 * 2.55 = 153
assertEqual(motorA, 153, "level drone: back-left motor matches throttle")
assertEqual(motorB, 153, "level drone: front-left motor matches throttle")
assertEqual(motorC, 153, "level drone: back-right motor matches throttle")
assertEqual(motorD, 153, "level drone: front-right motor matches throttle")

testGroup("when the drone tilts right, the left motors spin faster to correct")

airbit.resetPidState()
measuredRoll = 5
measuredPitch = 0
measuredYaw = 0
roll = 0
pitch = 0
yaw = 0
throttle = 60
rollPitchP = 0.9
rollPitchI = 0.004
rollPitchD = 15
yawP = 5
yawD = 70

airbit.stabilize()

// rollError = 0 - 5 = -5
// rollCorrection = -5*0.9 + 0 + -5*15 = -4.5 - 75 = -79.5
// motorA = round(153 + (-79.5) + 0 + 0) = 74
// motorC = round(153 - (-79.5) + 0 + 0) = 233
assertTrue(motorA < motorC, "right tilt: back-left motor slower than back-right")
assertTrue(motorB < motorD, "right tilt: front-left motor slower than front-right")

testGroup("motor speeds stay within safe limits even with extreme tilt")

airbit.resetPidState()
measuredRoll = 15
measuredPitch = 15
measuredYaw = 0
roll = -15
pitch = -15
yaw = 0
throttle = 100
rollPitchP = 0.9
rollPitchI = 0.004
rollPitchD = 15
yawP = 5
yawD = 70

airbit.stabilize()

assertTrue(motorA >= 0, "back-left motor never goes below zero")
assertTrue(motorA <= 255, "back-left motor never exceeds maximum")
assertTrue(motorB >= 0, "front-left motor never goes below zero")
assertTrue(motorB <= 255, "front-left motor never exceeds maximum")
assertTrue(motorC >= 0, "back-right motor never goes below zero")
assertTrue(motorC <= 255, "back-right motor never exceeds maximum")
assertTrue(motorD >= 0, "front-right motor never goes below zero")
assertTrue(motorD <= 255, "front-right motor never exceeds maximum")

// --- Battery Level Tests ---

testGroup("battery percentage matches the charge level")

// batteryLevel() calls batteryCalculation() first (reads analog pin, applies smoothing).
// On the simulator, analogReadPin returns 0, so batteryMillivoltsSmoothed converges toward 0.
// We can set batteryMillivoltsSmoothed directly and test the mapping by calling batteryLevel().
// Note: batteryLevel() calls batteryCalculation() which will modify batteryMillivoltsSmoothed.
// So we test the math directly: Math.map(voltage, 3400, 4200, 0, 100)
// These values match BATTERY_VOLTAGE_MIN (3400) and BATTERY_VOLTAGE_MAX (4200) in custom.ts

let expectedMin = Math.map(3400, 3400, 4200, 0, 100)
assertEqual(expectedMin, 0, "empty battery reads as 0%")

let expectedMax = Math.map(4200, 3400, 4200, 0, 100)
assertEqual(expectedMax, 100, "full battery reads as 100%")

let expectedMid = Math.map(3800, 3400, 4200, 0, 100)
assertEqual(expectedMid, 50, "half-charged battery reads as 50%")

// --- Stabilization Reset Tests ---
// Resetting stabilization clears all accumulated corrections and the measured spin angle.

testGroup("when stabilization is reset, the measured spin angle returns to zero")

measuredYaw = 45
airbit.resetPidState()
assertEqual(measuredYaw, 0, "after reset, measured spin angle is zero")

// --- All Tests Passed ---

serial.writeLine("=== ALL TESTS PASSED ===")
basic.showIcon(IconNames.Yes)
