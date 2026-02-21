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

function servo1_test () {
    pins.digitalWritePin(DigitalPin.P1, 1)
    control.waitMicros(1500 + roll * 10)
    pins.digitalWritePin(DigitalPin.P1, 0)
}
function JoystickDeadBand () {
    if (Math.abs(roll) < JOYSTICK_DEADBAND) {
        roll = 0
    }
    if (Math.abs(pitch) < JOYSTICK_DEADBAND) {
        pitch = 0
    }
}
function screen () {
    if (pins.analogReadPin(AnalogPin.P0) > CHARGING_THRESHOLD) {
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
    } else {
        if (mode == 0) {
            dots()
        }
        if (mode == 1) {
            led.plotBarGraph(
            airbit.batteryLevel(),
            100
            )
        }
        if (mode == 2) {
            basic.showNumber(airbit.batterymVolt())
        }
        if (mode == 3) {
            basic.showNumber(pins.analogReadPin(AnalogPin.P0))
        }
        if (mode == 4) {
            basic.showNumber(throttle)
        }
        if (mode == 5) {
            motorTest()
        }
        if (mode == 6) {
            basic.clearScreen()
            motorLed()
        }
    }
}
function mainLoop () {
    while (true) {
        // Read raw data from gyro and accelerometer
        airbit.IMU_sensorRead()
        // Find drone's absolute Roll, Pitch and Yaw angles with sensor fusion, gyro and accelerometer together.
        airbit.calculateAngles()
        basic.pause(1)
        lostSignalCheck()
        if (motorTesting == false) {
            // The "magic" algorithm that stabilises the drone based on setpoint angle and actual angle, finding the difference and chanring motor speed to compensate.
            airbit.stabilisePid()
        }
        // If upside down while armed, disable flying
        if (Math.abs(imuRoll) > FLIP_ANGLE_THRESHOLD && arm) {
            stable = false
        }
        // Only start motors if armed, stable, motor controller and gyro is operating
        if (arm && stable && (mcExists && gyroExists)) {
            if (throttle == 0) {
                // Idle speed of motors
                airbit.MotorSpeed(
                MOTOR_IDLE_SPEED,
                MOTOR_IDLE_SPEED,
                MOTOR_IDLE_SPEED,
                MOTOR_IDLE_SPEED
                )
            } else {
                airbit.MotorSpeed(
                motorA,
                motorB,
                motorC,
                motorD
                )
            }
        } else {
            // Clear registers for error compensation algorithms, do not keep errors from past flight.
            airbit.cleanReg()
            if (motorTesting) {
                airbit.MotorSpeed(
                motorA,
                motorB,
                motorC,
                motorD
                )
            } else {
                airbit.MotorSpeed(
                0,
                0,
                0,
                0
                )
            }
        }
        cpuTime = input.runningTime() - startTime
        startTime = input.runningTime()
    }
}
input.onButtonPressed(Button.A, function () {
    mode += -1
    if (mode < 0) {
        mode = DISPLAY_MODE_COUNT - 1
    }
})
function radioSendData () {
    radio.sendValue("p", rollPitchP)
    radio.sendValue("i", rollPitchI)
    radio.sendValue("d", rollPitchD)
    radio.sendValue("t", radioReceivedTime)
    radio.sendValue("R2", roll)
    radio.sendValue("yp", yawP)
    radio.sendValue("yd", yawD)
    radio.sendValue("v", batterymVoltSmooth)
    radio.sendValue("p0", pins.analogReadPin(AnalogPin.P0))
    basic.pause(TELEMETRY_INTERVAL)
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
function motorLed () {
    basic.clearScreen()
    led.plotBrightness(0, 4, motorA)
    led.plotBrightness(0, 0, motorB)
    led.plotBrightness(4, 4, motorC)
    led.plotBrightness(4, 0, motorD)
    led.plot(Math.map(imuRoll, -ANGLE_LIMIT, ANGLE_LIMIT, 0, 4), Math.map(imuPitch, -ANGLE_LIMIT, ANGLE_LIMIT, 4, 0))
}
radio.onReceivedValue(function (name, value) {
    radioReceivedTime = input.runningTime()
    if (name == "P") {
        pitch = expo(value) / PITCH_SCALE
        pitch = Math.constrain(pitch, -ANGLE_LIMIT, ANGLE_LIMIT)
    }
    if (name == "A") {
        arm = value
    }
    if (name == "R") {
        roll = expo(value) / ROLL_SCALE
        roll = Math.constrain(roll, -ANGLE_LIMIT, ANGLE_LIMIT)
    }
    if (name == "T") {
        throttle = value
        throttle = Math.constrain(throttle, 0, MAX_THROTTLE)
        if (batterymVoltSmooth < LOW_BATTERY_VOLTAGE) {
            throttle = Math.constrain(throttle, 0, LOW_BATTERY_THROTTLE)
        }
    }
    if (name == "Y") {
        yaw += value * YAW_SCALE
    }
})
function dots () {
    basic.clearScreen()
    led.plot(Math.map(roll, -ANGLE_LIMIT, ANGLE_LIMIT, 0, 4), Math.map(pitch, -ANGLE_LIMIT, ANGLE_LIMIT, 4, 0))
    led.plot(Math.map(yaw, -YAW_RANGE, YAW_RANGE, 0, 4), 4)
    if (arm) {
        led.plot(0, 0)
    }
    airbit.smartBar(0, throttle)
    airbit.smartBar(4, airbit.batteryLevel())
}
function lostSignalCheck () {
    // Failsafe makes only sense if already flying
    if (throttle > FAILSAFE_THROTTLE && arm) {
        if (input.runningTime() > radioReceivedTime + FAILSAFE_TIMEOUT_DESCEND) {
            roll = 0
            pitch = 0
            yaw = 0
            throttle = FAILSAFE_THROTTLE
        }
        if (input.runningTime() > radioReceivedTime + FAILSAFE_TIMEOUT_DISARM) {
            roll = 0
            pitch = 0
            yaw = 0
            throttle = 0
            arm = 0
        }
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
let soundStage = 0
let yaw = 0
let radioReceivedTime = 0
let startTime = 0
let cpuTime = 0
let motorTesting = false
let throttle = 0
let mode = 0
let pitch = 0
let roll = 0
let arm = 0
let expoFactor = 0
let expoSetting = 0
let motorD = 0
let motorB = 0
let motorC = 0
let motorA = 0
let yawD = 0
let yawP = 0
let rollPitchD = 0
let rollPitchI = 0
let rollPitchP = 0
let batterymVoltSmooth = 0
let imuRoll = 0
let imuPitch = 0
let stable = false
let gyroExists = false
let mcExists = false
let batteryVolt = 0
let imuYaw = 0
let baroExists = false
mcExists = false
gyroExists = false
stable = true
let radioGroup = RADIO_GROUP
imuPitch = 0
imuRoll = 0
batterymVoltSmooth = 3700
// Default: 0.7
rollPitchP = 0.9
rollPitchI = 0.004
// Default: 15
rollPitchD = 15
// Default: 4
yawP = 5
// Default: 10
yawD = 70
motorA = 0
motorC = 0
motorB = 0
motorD = 0
expoSetting = 2
expoFactor = EXPO_BASE * EXPO_BASE / (EXPO_BASE - EXPO_BASE / expoSetting)
radio.setGroup(radioGroup)
i2crr.setI2CPins(DigitalPin.P2, DigitalPin.P1)
basic.pause(100)
airbit.IMU_Start()
basic.pause(100)
airbit.PCA_Start()
basic.pause(100)
airbit.IMU_gyro_calibrate()
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
