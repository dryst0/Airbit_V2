// ============================================================================
// AIRBIT V2 — Drone Flight Controller
// ============================================================================
//
// HOW A DRONE FLIES
// -----------------
// A drone has 4 motors, one on each corner. Each motor spins a propeller.
// By spinning some motors faster than others, the drone can tilt and move:
//
//     Motor B (front-left)    Motor D (front-right)
//              \                /
//               \   micro:bit  /
//                +-----------+
//               /             \
//              /               \
//     Motor A (back-left)     Motor C (back-right)
//
// ROLL  = tilt left/right   (lean your body sideways)
// PITCH = tilt forward/back (lean your body forward)
// YAW   = spin left/right   (turn on the spot like a top)
//
// The pilot sends commands from a remote controller over radio.
// This code reads those commands, checks the drone's sensors,
// and adjusts the motor speeds to keep the drone stable in the air.
// ============================================================================

// --- Joystick ---
// The deadband ignores tiny joystick movements so the drone stays still
const JOYSTICK_DEADBAND = 5
// Maximum tilt angle in degrees the pilot can request
const ANGLE_LIMIT = 15
const YAW_RANGE = 30
// How much the joystick input is scaled down for each axis
const PITCH_SCALE = -3
const ROLL_SCALE = 3
const YAW_SCALE = 0.1

// --- Throttle ---
// Throttle controls how much power goes to all motors (0 = off, 100 = full)
const MAX_THROTTLE = 100
const LOW_BATTERY_THROTTLE = 75
// When armed but throttle is zero, motors spin very slowly so they're ready
const MOTOR_IDLE_SPEED = 5

// --- Safety ---
// If the drone tilts more than 90 degrees, something is very wrong — stop motors!
const FLIP_ANGLE_THRESHOLD = 90
// If we lose the radio signal, slowly descend then turn off
const FAILSAFE_THROTTLE = 65
const FAILSAFE_TIMEOUT_DESCEND = 3000  // Start descending after 3 seconds
const FAILSAFE_TIMEOUT_DISARM = 8000   // Turn off motors after 8 seconds

// --- Battery ---
// Battery voltage tells us how much charge is left (millivolts)
const LOW_BATTERY_VOLTAGE = 3400
const NORMAL_BATTERY_VOLTAGE = 3450
// These analog readings detect if the USB charger is plugged in
const CHARGING_THRESHOLD = 780
const CHARGING_COMPLETE_THRESHOLD = 950

// --- Exponential Curve ---
// The exponential curve makes the joystick less sensitive near the center for smoother control
const EXPO_BASE = 45

// --- Display ---
const DISPLAY_MODE_COUNT = 7
const TELEMETRY_INTERVAL = 5000  // Send debug info over radio every 5 seconds
const MOTOR_TEST_ITERATIONS = 50
const MOTOR_TEST_PAUSE = 20
const MOTOR_TEST_SPEED = 5

// --- Radio ---
// Both the drone and the remote must use the same radio group to talk
const RADIO_GROUP = 7

// Ignore tiny joystick movements — if the stick is barely moved, treat it as zero.
// WHY? Joysticks are never perfectly centered. Even when you let go, they might
// read 2 or 3 instead of exactly 0. Without a deadband, the drone would constantly
// drift because it thinks the pilot is giving a tiny command. The deadband creates
// a small "ignore zone" around the center where any reading is treated as zero.
function applyJoystickDeadband () {
    if (Math.abs(roll) < JOYSTICK_DEADBAND) {
        roll = 0
    }
    if (Math.abs(pitch) < JOYSTICK_DEADBAND) {
        pitch = 0
    }
}
// Check if the USB charger is plugged in by reading the voltage on pin P0
function isCharging (): boolean {
    return pins.analogReadPin(AnalogPin.P0) > CHARGING_THRESHOLD
}
// Show a battery icon that fills up while charging
function showChargingAnimation () {
    if (pins.analogReadPin(AnalogPin.P0) > CHARGING_COMPLETE_THRESHOLD) {
        basic.showIcon(IconNames.Yes)
        basic.showString("Charging finished!")
        return
    }
    basic.showLeds(`
        . . # . .
        . # # # .
        . # . # .
        . # . # .
        . # # # .
        `)
    basic.showLeds(`
        . . # . .
        . # # # .
        . # . # .
        . # # # .
        . # # # .
        `)
    basic.showLeds(`
        . . # . .
        . # # # .
        . # # # .
        . # # # .
        . # # # .
        `)
}
function showBatteryBar () {
    led.plotBarGraph(airbit.batteryLevel(), 100)
}
function showBatteryVoltage () {
    basic.showNumber(airbit.batteryMillivolts())
}
function showAnalogReading () {
    basic.showNumber(pins.analogReadPin(AnalogPin.P0))
}
function showThrottle () {
    basic.showNumber(throttle)
}
// Show motor speeds as brightness in each corner, with a dot showing the tilt angle
function showMotorLeds () {
    basic.clearScreen()
    led.plotBrightness(0, 4, motorA)
    led.plotBrightness(0, 0, motorB)
    led.plotBrightness(4, 4, motorC)
    led.plotBrightness(4, 0, motorD)
    led.plot(Math.map(measuredRoll, -ANGLE_LIMIT, ANGLE_LIMIT, 0, 4), Math.map(measuredPitch, -ANGLE_LIMIT, ANGLE_LIMIT, 4, 0))
}
// Pick what to show on the LED screen — use buttons A and B to switch modes
function screen () {
    if (isCharging()) {
        showChargingAnimation()
        return
    }
    switch (mode) {
        case 0: showJoystickPosition(); break
        case 1: showBatteryBar(); break
        case 2: showBatteryVoltage(); break
        case 3: showAnalogReading(); break
        case 4: showThrottle(); break
        case 5: motorTest(); break
        case 6: showMotorLeds(); break
    }
}
// Safety check: if the drone has flipped over (tilted past 90 degrees), disable it.
// WHY? If the drone flips upside down, the motors pushing "up" would now push it
// into the ground at full speed. Disabling the motors prevents damage and injury.
function checkStability () {
    if (Math.abs(measuredRoll) > FLIP_ANGLE_THRESHOLD && arm) {
        stable = false
    }
}
// Send the calculated motor speeds to the motor controller chip over I2C.
// Only fly if: armed, stable, and both the gyroscope and motor controller are working.
// WHY ALL THE CHECKS? Flying with a broken gyroscope means no balance — instant crash.
// Flying while disarmed or flipped would be dangerous. These checks are like a
// pre-flight checklist that pilots go through before every takeoff.
// When not ready to fly, reset stabilization and either pass through motor test speeds or stop
function handleMotorsNotReady () {
    airbit.resetPidState()
    if (motorTesting) {
        airbit.setMotorSpeeds(motorA, motorB, motorC, motorD)
        return
    }
    airbit.setMotorSpeeds(0, 0, 0, 0)
}
function updateMotors () {
    if (!arm || !stable || !motorControllerExists || !gyroExists) {
        handleMotorsNotReady()
        return
    }
    if (throttle == 0) {
        airbit.setMotorSpeeds(MOTOR_IDLE_SPEED, MOTOR_IDLE_SPEED, MOTOR_IDLE_SPEED, MOTOR_IDLE_SPEED)
        return
    }
    airbit.setMotorSpeeds(motorA, motorB, motorC, motorD)
}
function trackLoopTime () {
    cpuTime = input.runningTime() - startTime
    startTime = input.runningTime()
}
// The main flight loop — this runs hundreds of times per second.
// WHY SO FAST? The drone is inherently unstable — without constant corrections,
// it would tip over in a fraction of a second (like balancing a pencil on your
// finger). The faster we read the sensors and adjust the motors, the smoother
// and more stable the flight. Each cycle: read sensors → calculate angles →
// run PID stabilization → check safety → update motors.
// Only run PID stabilization when not in motor test mode
function stabilizeIfFlying () {
    if (motorTesting) return
    airbit.stabilize()
}
function mainLoop () {
    while (true) {
        airbit.readImuSensors()
        airbit.calculateAngles()
        basic.pause(1)
        lostSignalCheck()
        stabilizeIfFlying()
        checkStability()
        updateMotors()
        trackLoopTime()
    }
}
input.onButtonPressed(Button.A, function () {
    mode += -1
    if (mode < 0) {
        mode = DISPLAY_MODE_COUNT - 1
    }
})
// Send telemetry (debug data) back to the remote controller over radio.
// WHY? During testing, the pilot needs to see what the drone is "thinking" —
// PID settings, battery voltage, signal timing — without plugging in a cable.
// We only send every 5 seconds to avoid flooding the radio channel.
function radioSendData () {
    if (input.runningTime() - lastTelemetryTime < TELEMETRY_INTERVAL) return
    lastTelemetryTime = input.runningTime()
    radio.sendValue("p", rollPitchP)
    radio.sendValue("i", rollPitchI)
    radio.sendValue("d", rollPitchD)
    radio.sendValue("t", radioReceivedTime)
    radio.sendValue("R2", roll)
    radio.sendValue("yp", yawP)
    radio.sendValue("yd", yawD)
    radio.sendValue("v", batteryMillivoltsSmoothed)
    radio.sendValue("p0", pins.analogReadPin(AnalogPin.P0))
}
input.onButtonPressed(Button.AB, function () {
    mode = 0
})
input.onGesture(Gesture.ScreenDown, function () {
    stable = false
})
input.onButtonPressed(Button.B, function () {
    mode += 1
    if (mode > DISPLAY_MODE_COUNT - 1) {
        mode = 0
    }
})
// Process the pitch (forward/back tilt) command from the remote.
// The raw joystick value goes through the expo curve (for smoother control),
// gets divided by PITCH_SCALE to convert to degrees, then clamped to a safe
// maximum tilt angle so the drone can't flip over from a joystick command.
function receivePitch (value: number) {
    pitch = applyExponentialCurve(value) / PITCH_SCALE
    pitch = Math.constrain(pitch, -ANGLE_LIMIT, ANGLE_LIMIT)
}
// Process the roll (left/right tilt) command from the remote
function receiveRoll (value: number) {
    roll = applyExponentialCurve(value) / ROLL_SCALE
    roll = Math.constrain(roll, -ANGLE_LIMIT, ANGLE_LIMIT)
}
// Process the throttle (up/down power) command. If battery is low, limit the max power.
// WHY LIMIT? A weak battery can't deliver full power safely. Pushing it too hard
// can cause a sudden voltage drop, making the drone fall out of the sky. Capping
// the throttle at 75% when the battery is low gives the pilot a gentler warning
// and keeps enough voltage reserve for a safe landing.
function receiveThrottle (value: number) {
    throttle = Math.constrain(value, 0, MAX_THROTTLE)
    if (batteryMillivoltsSmoothed < LOW_BATTERY_VOLTAGE) {
        throttle = Math.constrain(throttle, 0, LOW_BATTERY_THROTTLE)
    }
}
// Arm or disarm the drone. "Arming" is like turning the ignition key in a car.
// WHY HAVE AN ARM SWITCH? Safety! You don't want the motors to spin just because
// the throttle stick is bumped. The pilot must deliberately flip the arm switch
// before the motors will respond. When re-armed, we reset the stability flag so
// the drone can fly again after being picked up from a flip.
function receiveArm (value: number) {
    if (!arm && value) {
        stable = true
    }
    arm = value
}
// When the remote sends a radio command, figure out what it is and handle it
radio.onReceivedValue(function (name, value) {
    radioReceivedTime = input.runningTime()
    switch (name) {
        case "P": receivePitch(value); break
        case "A": receiveArm(value); break
        case "R": receiveRoll(value); break
        case "T": receiveThrottle(value); break
        case "Y": yaw += value * YAW_SCALE; break
    }
})
// Default display: show joystick position as a dot, throttle bar on left, battery on right
function showJoystickPosition () {
    basic.clearScreen()
    led.plot(Math.map(roll, -ANGLE_LIMIT, ANGLE_LIMIT, 0, 4), Math.map(pitch, -ANGLE_LIMIT, ANGLE_LIMIT, 4, 0))
    led.plot(Math.map(yaw, -YAW_RANGE, YAW_RANGE, 0, 4), 4)
    if (arm) {
        led.plot(0, 0)
    }
    airbit.smartBar(0, throttle)
    airbit.smartBar(4, airbit.batteryLevel())
}
function resetFlightInputs () {
    roll = 0
    pitch = 0
    yaw = 0
}
// Failsafe: if we haven't heard from the remote in a while, the drone is on its own!
// WHY? If the pilot's remote loses connection (battery dies, goes out of range),
// the drone would keep flying with its last command forever — dangerous! This
// safety check measures how long it's been since the last radio message:
//   - After 3 seconds of silence: reduce throttle and start descending slowly
//   - After 8 seconds: shut off motors completely
// This way a disconnected drone lands itself instead of flying away.
function lostSignalCheck () {
    if (throttle <= FAILSAFE_THROTTLE || !arm) return
    const timeSinceLastSignal = input.runningTime() - radioReceivedTime
    if (timeSinceLastSignal > FAILSAFE_TIMEOUT_DISARM) {
        resetFlightInputs()
        throttle = 0
        arm = 0
        return
    }
    if (timeSinceLastSignal > FAILSAFE_TIMEOUT_DESCEND) {
        resetFlightInputs()
        throttle = FAILSAFE_THROTTLE
    }
}
// Test each motor one at a time with a spinning dot animation to verify they all work
// Spin one motor while showing a rotating dot on the LED screen
function testSingleMotor (dotX: number, dotY: number, rotationSpeed: number) {
    for (let index = 0; index < MOTOR_TEST_ITERATIONS; index++) {
        basic.clearScreen()
        airbit.rotateDot(dotX, dotY, 1, rotationSpeed)
        basic.pause(MOTOR_TEST_PAUSE)
    }
}
function motorTest () {
    motorA = 0
    motorB = 0
    motorC = 0
    motorD = 0
    motorTesting = true
    motorB = MOTOR_TEST_SPEED
    testSingleMotor(1, 1, 10)
    motorB = 0
    motorD = MOTOR_TEST_SPEED
    testSingleMotor(3, 1, -10)
    motorD = 0
    motorC = MOTOR_TEST_SPEED
    testSingleMotor(3, 3, 10)
    motorC = 0
    motorA = MOTOR_TEST_SPEED
    testSingleMotor(1, 3, -10)
    motorA = 0
    motorTesting = false
}
// Play sounds at different battery levels to warn the pilot
function sounds () {
    if (arm && soundStage == 0) {
        soundExpression.giggle.playUntilDone()
        soundStage = 1
    }
    if (airbit.batteryLevel() < 50 && soundStage == 1) {
        soundExpression.slide.playUntilDone()
        soundStage = 2
    }
    if (airbit.batteryLevel() < 20 && soundStage == 2) {
        soundExpression.sad.playUntilDone()
        soundStage = 3
    }
}
// Exponential curve: makes the joystick less sensitive near the center.
// WHY? Without this, the drone would be twitchy and hard to control.
// Imagine steering a car where a tiny turn of the wheel makes you swerve —
// you'd crash instantly. The expo curve makes small joystick movements produce
// very gentle drone movements (for precise hovering), while big joystick
// movements still give full speed (for fast maneuvers). The math uses a
// quadratic (x²) formula: small inputs get squared into even smaller outputs,
// but large inputs stay large.
function applyExponentialCurve (joystickValue: number) {
    if (joystickValue >= 0) {
        return joystickValue / expoSetting + joystickValue * joystickValue / expoFactor
    }
    return joystickValue / expoSetting - joystickValue * joystickValue / expoFactor
}
// --- Flight Input State ---
// These are the commands the pilot sends from the remote controller
let roll = 0       // Left/right tilt angle requested by the pilot
let pitch = 0      // Forward/back tilt angle requested by the pilot
let yaw = 0        // Spin direction requested by the pilot
let throttle = 0   // Motor power (0-100%)
let arm = 0        // 0 = motors disabled (safe), 1 = motors enabled (ready to fly)

// --- Motor Output ---
// The speed of each motor (0-255), calculated by the stabilization algorithm
let motorA = 0     // Back-left
let motorB = 0     // Front-left
let motorC = 0     // Back-right
let motorD = 0     // Front-right
let motorTesting = false

// --- Measured Angles ---
// The actual angles the drone is tilted at right now, measured by the gyroscope
let measuredRoll = 0    // Actual left/right tilt (degrees)
let measuredPitch = 0   // Actual forward/back tilt (degrees)
let measuredYaw = 0     // Actual spin angle (degrees)

// --- PID Tuning Parameters ---
// PID stands for Proportional, Integral, Derivative — it's a control algorithm.
// Think of it like balancing a ball on a plate:
//   P (Proportional) = how hard you tilt the plate when the ball is off-center
//   I (Integral)     = if the ball keeps drifting one way, gradually push harder
//   D (Derivative)   = if the ball is moving fast, slow down the correction
let rollPitchP = 0.9
let rollPitchI = 0.004
let rollPitchD = 15
let yawP = 5
let yawD = 70

// --- Exponential Curve ---
let expoSetting = 2
let expoFactor = EXPO_BASE * EXPO_BASE / (EXPO_BASE - EXPO_BASE / expoSetting)

// --- Battery ---
// Smoothed battery voltage — avoids jumpy readings (millivolts)
let batteryMillivoltsSmoothed = 3700

// --- Hardware Status ---
let motorControllerExists = false    // Motor controller chip found?
let gyroExists = false  // Gyroscope sensor found?
let stable = true       // Is the drone right-side up?

// --- Display ---
let mode = 0         // Which screen to show (cycle with A/B buttons)
let soundStage = 0   // Tracks which battery warning sound was last played

// --- Timing ---
let startTime = 0
let cpuTime = 0
let radioReceivedTime = 0
let lastTelemetryTime = 0

// ============================================================================
// STARTUP SEQUENCE
// ============================================================================
// Set up the radio so we can talk to the remote controller
radio.setGroup(RADIO_GROUP)
// Tell the micro:bit which physical pins connect to the I2C data (SDA) and
// clock (SCL) wires. The drone board uses pins P2 and P1 instead of the defaults.
i2crr.setI2CPins(DigitalPin.P2, DigitalPin.P1)
// Wake up and test the gyroscope (shows "G" if found)
basic.pause(100)
airbit.startImu()
// Wake up and test the motor controller (shows "M" if found)
basic.pause(100)
airbit.startMotorController()
// Calibrate the gyroscope — the drone must be sitting still on a flat surface!
// (shows "C" while calibrating, then a checkmark when done)
basic.pause(100)
airbit.calibrateGyro()
// Safety: don't start if the drone was accidentally left armed
while (arm) {
    basic.showString("Disarm!")
}
// ============================================================================
// FOREVER LOOPS — these run continuously after startup
// ============================================================================

// Show the right thing on the LED screen based on battery level and stability
function updateDisplay () {
    if (!stable) {
        basic.showString("Tilted. Please reset.")
        return
    }
    if (batteryMillivoltsSmoothed > NORMAL_BATTERY_VOLTAGE) {
        screen()
        return
    }
    if (batteryMillivoltsSmoothed > LOW_BATTERY_VOLTAGE) {
        basic.showLeds(`
            . . # . .
            . # . # .
            . # . # .
            . # . # .
            . # # # .
            `)
        return
    }
    basic.showLeds(`
        . . # . .
        . # . # .
        . # . # .
        . # . # .
        . # # # .
        `)
    basic.showLeds(`
        . . . . .
        . . . . .
        . . . . .
        . . . . .
        . . . . .
        `)
}
// Display loop: show status on the LED screen
basic.forever(function () {
    updateDisplay()
})
// Telemetry loop: send debug data to the remote over radio
basic.forever(function () {
    radioSendData()
})
// Battery loop: keep measuring the battery voltage
basic.forever(function () {
    airbit.batteryCalculation()
})
// Flight loop: the core loop that reads sensors and controls the motors
basic.forever(function () {
    mainLoop()
})
