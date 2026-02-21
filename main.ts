// --- Joystick ---
const JOYSTICK_DEADBAND = 5
const ANGLE_LIMIT = 15
const YAW_RANGE = 30
const PITCH_SCALE = -3
const ROLL_SCALE = 3
const YAW_SCALE = 0.1

// --- Throttle ---
const MAX_THROTTLE = 100
const LOW_BATTERY_THROTTLE = 75
const MOTOR_IDLE_SPEED = 5

// --- Safety ---
const FLIP_ANGLE_THRESHOLD = 90
const FAILSAFE_THROTTLE = 65
const FAILSAFE_TIMEOUT_DESCEND = 3000
const FAILSAFE_TIMEOUT_DISARM = 8000

// --- Battery ---
const LOW_BATTERY_VOLTAGE = 3400
const NORMAL_BATTERY_VOLTAGE = 3450
const CHARGING_THRESHOLD = 780
const CHARGING_COMPLETE_THRESHOLD = 950

// --- Expo ---
const EXPO_BASE = 45

// --- Display ---
const DISPLAY_MODE_COUNT = 7
const TELEMETRY_INTERVAL = 5000
const MOTOR_TEST_ITERATIONS = 50
const MOTOR_TEST_PAUSE = 20
const MOTOR_TEST_SPEED = 5

// --- Radio ---
const RADIO_GROUP = 7

function applyJoystickDeadband () {
    if (Math.abs(roll) < JOYSTICK_DEADBAND) {
        roll = 0
    }
    if (Math.abs(pitch) < JOYSTICK_DEADBAND) {
        pitch = 0
    }
}
function isCharging (): boolean {
    return pins.analogReadPin(AnalogPin.P0) > CHARGING_THRESHOLD
}
function showChargingAnimation () {
    if (pins.analogReadPin(AnalogPin.P0) > CHARGING_COMPLETE_THRESHOLD) {
        basic.showIcon(IconNames.Yes)
        basic.showString("Charging finished!")
    } else {
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
function showMotorLeds () {
    basic.clearScreen()
    led.plotBrightness(0, 4, motorA)
    led.plotBrightness(0, 0, motorB)
    led.plotBrightness(4, 4, motorC)
    led.plotBrightness(4, 0, motorD)
    led.plot(Math.map(imuRoll, -ANGLE_LIMIT, ANGLE_LIMIT, 0, 4), Math.map(imuPitch, -ANGLE_LIMIT, ANGLE_LIMIT, 4, 0))
}
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
function checkStability () {
    if (Math.abs(imuRoll) > FLIP_ANGLE_THRESHOLD && arm) {
        stable = false
    }
}
function updateMotors () {
    if (arm && stable && mcExists && gyroExists) {
        if (throttle == 0) {
            airbit.setMotorSpeeds(MOTOR_IDLE_SPEED, MOTOR_IDLE_SPEED, MOTOR_IDLE_SPEED, MOTOR_IDLE_SPEED)
        } else {
            airbit.setMotorSpeeds(motorA, motorB, motorC, motorD)
        }
    } else {
        airbit.resetPidState()
        if (motorTesting) {
            airbit.setMotorSpeeds(motorA, motorB, motorC, motorD)
        } else {
            airbit.setMotorSpeeds(0, 0, 0, 0)
        }
    }
}
function trackLoopTime () {
    cpuTime = input.runningTime() - startTime
    startTime = input.runningTime()
}
function mainLoop () {
    while (true) {
        airbit.readImuSensors()
        airbit.calculateAngles()
        basic.pause(1)
        lostSignalCheck()
        if (!motorTesting) {
            airbit.stabilize()
        }
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
    radio.sendValue("v", batterymVoltSmooth)
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
function receivePitch (value: number) {
    pitch = expo(value) / PITCH_SCALE
    pitch = Math.constrain(pitch, -ANGLE_LIMIT, ANGLE_LIMIT)
}
function receiveRoll (value: number) {
    roll = expo(value) / ROLL_SCALE
    roll = Math.constrain(roll, -ANGLE_LIMIT, ANGLE_LIMIT)
}
function receiveThrottle (value: number) {
    throttle = Math.constrain(value, 0, MAX_THROTTLE)
    if (batterymVoltSmooth < LOW_BATTERY_VOLTAGE) {
        throttle = Math.constrain(throttle, 0, LOW_BATTERY_THROTTLE)
    }
}
function receiveArm (value: number) {
    if (!arm && value) {
        stable = true
    }
    arm = value
}
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
function lostSignalCheck () {
    if (throttle <= FAILSAFE_THROTTLE || !arm) return
    const timeSinceLastSignal = input.runningTime() - radioReceivedTime
    if (timeSinceLastSignal > FAILSAFE_TIMEOUT_DISARM) {
        resetFlightInputs()
        throttle = 0
        arm = 0
    } else if (timeSinceLastSignal > FAILSAFE_TIMEOUT_DESCEND) {
        resetFlightInputs()
        throttle = FAILSAFE_THROTTLE
    }
}
function motorTest () {
    motorA = 0
    motorB = 0
    motorC = 0
    motorD = 0
    motorTesting = true
    motorB = MOTOR_TEST_SPEED
    for (let index = 0; index < MOTOR_TEST_ITERATIONS; index++) {
        basic.clearScreen()
        airbit.rotateDot(
        1,
        1,
        1,
        10
        )
        basic.pause(MOTOR_TEST_PAUSE)
    }
    motorB = 0
    motorD = MOTOR_TEST_SPEED
    for (let index = 0; index < MOTOR_TEST_ITERATIONS; index++) {
        basic.clearScreen()
        airbit.rotateDot(
        3,
        1,
        1,
        -10
        )
        basic.pause(MOTOR_TEST_PAUSE)
    }
    motorD = 0
    motorC = MOTOR_TEST_SPEED
    for (let index = 0; index < MOTOR_TEST_ITERATIONS; index++) {
        basic.clearScreen()
        airbit.rotateDot(
        3,
        3,
        1,
        10
        )
        basic.pause(MOTOR_TEST_PAUSE)
    }
    motorC = 0
    motorA = MOTOR_TEST_SPEED
    for (let index = 0; index < MOTOR_TEST_ITERATIONS; index++) {
        basic.clearScreen()
        airbit.rotateDot(
        1,
        3,
        1,
        -10
        )
        basic.pause(MOTOR_TEST_PAUSE)
    }
    motorA = 0
    motorTesting = false
}
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
function expo (inp: number) {
    if (inp >= 0) {
        return inp / expoSetting + inp * inp / expoFactor
    } else {
        return inp / expoSetting - inp * inp / expoFactor
    }
}
// --- Flight Input State ---
let roll = 0
let pitch = 0
let yaw = 0
let throttle = 0
let arm = 0

// --- Motor Output ---
let motorA = 0
let motorB = 0
let motorC = 0
let motorD = 0
let motorTesting = false

// --- IMU Angles ---
let imuRoll = 0
let imuPitch = 0
let imuYaw = 0

// --- PID Tuning Parameters ---
let rollPitchP = 0.9
let rollPitchI = 0.004
let rollPitchD = 15
let yawP = 5
let yawD = 70

// --- Expo Curve ---
let expoSetting = 2
let expoFactor = EXPO_BASE * EXPO_BASE / (EXPO_BASE - EXPO_BASE / expoSetting)

// --- Battery ---
let batterymVoltSmooth = 3700

// --- Hardware Status ---
let mcExists = false
let gyroExists = false
let stable = true

// --- Display ---
let mode = 0
let soundStage = 0

// --- Timing ---
let startTime = 0
let cpuTime = 0
let radioReceivedTime = 0
let lastTelemetryTime = 0
radio.setGroup(RADIO_GROUP)
i2crr.setI2CPins(DigitalPin.P2, DigitalPin.P1)
basic.pause(100)
airbit.startImu()
basic.pause(100)
airbit.startMotorController()
basic.pause(100)
airbit.calibrateGyro()
while (arm) {
    basic.showString("Disarm!")
}
basic.forever(function () {
    if (stable == false) {
        basic.showString("Tilted. Please reset.")
    } else if (batterymVoltSmooth > NORMAL_BATTERY_VOLTAGE) {
        screen()
    } else if (batterymVoltSmooth > LOW_BATTERY_VOLTAGE) {
        basic.showLeds(`
            . . # . .
            . # . # .
            . # . # .
            . # . # .
            . # # # .
            `)
    } else {
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
})
basic.forever(function () {
    radioSendData()
})
basic.forever(function () {
    airbit.batteryCalculation()
})
basic.forever(function () {
    mainLoop()
})
