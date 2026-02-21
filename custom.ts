
/**
* Use this file to define custom export functions and blocks.
* Read more at https://makecode.microbit.org/blocks/custom
*/

/**
 * Custom blocks
 */
//% weight=100 color=#0fbc11 icon=""
namespace airbit {


    /**
    * Draw a vertical bar with gradients for prescicion
    * X = 0..4 x position on screen, amount = 0..100
    */

    //% blockID=airbit_smart_bar
    //% block="Smart Bar $x $amount"
    //% group='Screen'
    //% x.min = 0 x.max=4
    //% amount.min = 0 amount.max = 100

    export function smartBar(x: number, amount: number) {
        for (let index = 0; index <= amount / 20; index++) {
            led.plot(x, 4 - index)
        }
        led.plotBrightness(x, 4 - Math.floor(amount / 20), 12.75 * (amount % 20))
    }

    /**
     * Initialise Barometer
     */

    //% blockID=airbit_start_baro
    //% block="Start Barometer"
    //% group='Control'
    export function baroStart() {
        // Soft reset
        pins.i2cWriteNumber(
            BARO_REG_SLAVEADR,
            32861,
            NumberFormat.UInt16BE,
            true
        )
        basic.pause(10)
        pins.i2cWriteNumber(
            BARO_REG_SLAVEADR,
            61384,
            NumberFormat.UInt16BE,
            true
        )
        BARO_return = pins.i2cReadNumber(BARO_REG_SLAVEADR, NumberFormat.UInt16LE, true)
        if (BARO_return) {
            basic.showString("B")
        } else {
            basic.showString("No Baro", 50)
        }
    }


    /**
     * Erase PID registers
     */

    //% blockID=airbit_clean_reg
    //% block="Clean Registers"
    //% group='Control'

    export function cleanReg() {
        rollDiff = 0
        pitchDiff = 0
        lastRollDiff = 0
        lastPitchDiff = 0
        lastYawDiff = 0
        rollIdiff = 0
        pitchIdiff = 0
        yawIdiff = 0
        yawDiff = 0
        yawDdiff = 0
        pitchDdiff = 0
        rollDdiff = 0
        imuYaw = 0
        gyroZdelta = 0
        yaw = 0
        rollCorrection = 0
        pitchCorrection = 0

    }

    /**
     * Battery level in %
     */
    //% blockID=airbit_battery_level
    //% block="Battery Level"
    //% group='Battery management'

    export function batteryLevel() {
        batteryCalculation()
        return Math.map(batterymVoltSmooth, 3400, 4200, 0, 100)
    }


    /**
    *   Battery calculation with smoothing (low pass filter)
    */

    //% blockID=airbit_battery_calculation
    //% block="Battery Calculation"
    //% group='Battery management'

    export function batteryCalculation() {
        batterymVoltSmooth = Math.round(pins.analogReadPin(AnalogPin.P0) * BATTERY_FACTOR * 0.1 + batterymVoltSmooth * 0.9)

    }

    /**
       Battery calculation (no smoothing) 
    */

    //% blockID=airbit_battery_calculation_simple
    //% block="Battery milliVolts"
    //% group='Battery management'

    export function batterymVolt() {
        return Math.round(pins.analogReadPin(AnalogPin.P0) * BATTERY_FACTOR)

    }




    /**
     * Read from the motor controller
     */

    //% blockID=airbit_read_pca
    //% block="Read Motor Controller"
    //% group='System'

    export function readPCA(num: number) {
        pins.i2cWriteNumber(
            PCA_REG_SLAVEADR,
            num,
            NumberFormat.UInt8BE,
            true
        )
        return pins.i2cReadNumber(PCA_REG_SLAVEADR, NumberFormat.UInt8BE, false)
    }



    export function radioSend() {
        radio.sendValue("B", batterymVoltSmooth)
        radio.sendValue("G", input.acceleration(Dimension.Z))
        radio.sendValue("Te", input.temperature())
        radio.sendValue("Rd", Math.round(imuRoll))
        radio.sendValue("Pd", Math.round(imuPitch))
    }

    /**
     * Calculate the drone's Roll, Pitch and Roll angles from raw data.
     */

    //% blockID=airbit_calculate_angles
    //% block="Calculate Angles"
    //% group='Control'

    export function calculateAngles() {
        looptime = input.runningTime() - oldTime
        oldTime = input.runningTime()
        accPitch = (-57.295 * Math.atan2(accY, accZ)) - accPitchOffset
        accRoll = (-57.295 * Math.atan2(accX, accZ)) - accRollOffset
        // Degrees away from desired angle
        gyroXdelta = (gyroX - gyroXcalibration) * looptime * -0.00000762939
        gyroYdelta = (gyroY - gyroYcalibration) * looptime * 0.00000762939
        gyroZdelta = (gyroZ - gyroZcalibration) * looptime * -0.00000762939
        imuRoll = (gyroYdelta + imuRoll) * 0.99 + accRoll * 0.01
        imuPitch = (gyroXdelta + imuPitch) * 0.99 + accPitch * 0.01
        imuYaw = gyroZdelta + imuYaw
    }

    /** 
     * 
     * Plot a rotating dot
     * xPos and yPos is the center point 0..4
     * Radius 1..4 (size)
     * Speed -100..100, use negative value for counter clock rotation
    */
    //% blockID=airbit_rotation_dot
    //% block="Rotation dot $xPos $yPos $radius $speed"
    //% xPos.min=0 xPos.max=4 xPos.dfl=2
    //% yPos.min=0 yPos.max=4 yPos.dfl=2
    //% radius.min=1 radius.max=4 radius.dfl=2
    //% speed.min=-100 speed.max=100 speed.dfl=10
    //% group='Screen'

    export function rotateDot(xPos: number, yPos: number, radius: number, speed: number) {
        led.plot(xPos + 0.5 + (radius+0.5) * Math.cos(input.runningTime() / 10000 * 6.283 * speed), yPos + 0.5 + (radius+0.5) * Math.sin(input.runningTime() / 10000 * 6.283 * speed))
    }



    /**
        Control the individual speed of each motor.
     */
    //% blockID=airbit_motor_speed
    //% block="Motor Speed $m0 $m1 $m2 $m3"
    //% m0.min=0 m0.max=255
    //% m1.min=0 m1.max=255
    //% m2.min=0 m2.max=255
    //% m3.min=0 m3.max=255

    //% group='Control'

    export function MotorSpeed(m0: number, m1: number, m2: number, m3: number) {
        pins.i2cWriteNumber(
            PCA_REG_SLAVEADR,
            PCA_pwm0 << 8 | m3,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            PCA_REG_SLAVEADR,
            PCA_pwm1 << 8 | m2,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            PCA_REG_SLAVEADR,
            PCA_pwm2 << 8 | m1,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            PCA_REG_SLAVEADR,
            PCA_pwm3 << 8 | m0,
            NumberFormat.UInt16BE,
            false
        )
    }


    /*
       Start and setup the Gyro/Accelereometer sensor
    */

    //% blockID=airbit_start_imu
    //% block="Start Gyro/Acc"
    //% group='Control'

    export function IMU_Start() {
        // Full reset chip (H_RESET, internal 20MHz clock)
        pins.i2cWriteNumber(
            IMU_REG_ADDRESS,
            IMU_PWR_MGMT_1 << 8 | 0x80,
            NumberFormat.UInt16BE,
            false
        )
        basic.pause(500)
        pins.i2cWriteNumber(
            IMU_REG_ADDRESS,
            IMU_WHO_AM_I,
            NumberFormat.UInt8BE,
            true
        )
        gyroReturnId = pins.i2cReadNumber(IMU_REG_ADDRESS, NumberFormat.Int16BE, false)
        // basic.showNumber(IMU_Return >> 8)
        basic.clearScreen()
        if (gyroReturnId >> 8 > 0) {
            basic.showString("G")
            gyroExists = true
        } else {
            basic.showString("NG", 50)
            gyroExists = false
        }
        // set clock to internal PLL
        pins.i2cWriteNumber(
            IMU_REG_ADDRESS,
            IMU_PWR_MGMT_1 << 8 | 0x01,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            IMU_REG_ADDRESS,
            IMU_SIGNAL_PATH_RESET << 8 | 0x07,
            NumberFormat.UInt16BE,
            false
        )
        // Disable FIFO
        pins.i2cWriteNumber(
            IMU_REG_ADDRESS,
            IMU_USER_CTRL << 8 | 0x00,
            NumberFormat.UInt16BE,
            false
        )
        // Gyro filter setting to 0 (250 Hz), 1 (176 Hz),  2 (92 Hz), 3 (41 Hz)
        pins.i2cWriteNumber(
            IMU_REG_ADDRESS,
            IMU_REG_CONFIG << 8 | 0,
            NumberFormat.UInt16BE,
            false
        )
        // Acc filter setting to 3 (44.8 Hz), 4 (21,2 Hz), 5 (10.2 Hz)
        pins.i2cWriteNumber(
            IMU_REG_ADDRESS,
            IMU_ACCEL_CONFIG_2 << 8 | 5,
            NumberFormat.UInt16BE,
            false
        )
    }



    /*
      Write to the motor controller
    */

    //% blockID=airbit_write_pca
    //% block="Write PCA"
    //% group='System'

    export function PCA_Write(register: number, value: number) {
        pins.i2cWriteNumber(
            PCA_REG_SLAVEADR,
            register << 8 | value,
            NumberFormat.UInt16BE,
            false
        )
    }


    /**
     * Read gyro and acceleration from sensor
     */

    //% blockID=airbit_read_imu
    //% block="Read Gyro/Acc"
    //% group='Control'

    export function IMU_sensorRead() {
        pins.i2cWriteNumber(
            IMU_REG_ADDRESS,
            67,
            NumberFormat.Int8LE,
            true
        )
        gyroX = pins.i2cReadNumber(104, NumberFormat.Int16BE, true)
        gyroY = pins.i2cReadNumber(104, NumberFormat.Int16BE, true)
        gyroZ = pins.i2cReadNumber(104, NumberFormat.Int16BE, false)
        pins.i2cWriteNumber(
            104,
            59,
            NumberFormat.Int8LE,
            true
        )
        accX = pins.i2cReadNumber(104, NumberFormat.Int16BE, true)
        accY = pins.i2cReadNumber(104, NumberFormat.Int16BE, true)
        accZ = pins.i2cReadNumber(104, NumberFormat.Int16BE, false)
    }



    // Mode2:
    // Totem pole:
    // Inverted = %10101(21)
    // Non-inverted = %00101(5)
    // 
    // Open Drain:
    // Inverted = %10001(17)
    // Non-inverted = %00001(1)

    /**
    * Setup motor controller
    */

    //% blockID=airbit_start_pca
    //% block="Start Motor Controller"
    //% group='Control'

    export function PCA_Start() {
        PCA_Write(PCA_REG_MODE1, 128)
        PCA_Write(PCA_REG_MODE2, PCA_REG_MODE2_CONFIG)
        // Mode2:Inverted, Totem pole on = %10101(21), Non-inverted = %00101(5)
        // Mode2:Inverted, Open drain = %10001(17), Non-inverted = %00001(1)
        PCA_Write(PCA_REG_LEDUOT, 170)

        MotorSpeed(0, 0, 0, 0)     // Zero out motor speed 
        // Self test to see if data reg can be read.
        pins.i2cWriteNumber(
            PCA_REG_SLAVEADR,
            PCA_REG_MODE2,
            NumberFormat.UInt8BE,
            true
        )
        mcReturnId = pins.i2cReadNumber(PCA_REG_SLAVEADR, NumberFormat.UInt8BE, false)
        basic.clearScreen()
        if (mcReturnId) {
            basic.showString("M")
            mcExists = true
        } else {
            basic.showString("No PCA!", 50)
            mcExists = false
        }
    }




    /**
    * Calibrate the gyro and accelerometer
    */
    //% blockID=airbit_calibrate_gyro
    //% block="Calibrate Gyro/Acc"
    //% group='Control'

    export function IMU_gyro_calibrate() {
        gyroXcalibration = 0
        gyroYcalibration = 0
        gyroZcalibration = 0
        basic.showString("C")
        for (let index = 0; index < 100; index++) {
            IMU_sensorRead()
            gyroXcalibration += gyroX
            gyroYcalibration += gyroY
            gyroZcalibration += gyroZ
            basic.pause(5)
        }
        gyroXcalibration = gyroXcalibration / 100
        gyroYcalibration = gyroYcalibration / 100
        gyroZcalibration = gyroZcalibration / 100
        accPitch = -57.295 * Math.atan2(accY, accZ)
        accRoll = -57.295 * Math.atan2(accX, accZ)
        accPitchOffset = accPitch
        accRollOffset = accRoll

        basic.showIcon(IconNames.Yes)
    }




    /**
     * Use PID algorithm to generate the four motor speeds 
     */

    //% blockID=airbit_stabilise_pid
    //% block="Stabilise PID"
    //% group='Control'


    export function stabilisePid() {

        rollDiff = roll - imuRoll
        pitchDiff = pitch - imuPitch      // Reversing the pitch
        yawDiff = yaw - imuYaw
        rollDdiff = rollDiff - lastRollDiff
        pitchDdiff = pitchDiff - lastPitchDiff
        yawDdiff = yawDiff - lastYawDiff

        lastRollDiff = rollDiff
        lastPitchDiff = pitchDiff
        lastYawDiff = yawDiff

        let iRange = 5      //  Maximal error that will increase Roll and Pitch integral
        let iLimit = 4      //  Maximal correcton that can be added by integral
        let yawLimit = 50   //  Maximal yaw correction 
       
        if (throttle > 50) {    // Prevent windup before flight

            if (rollDiff > - iRange && rollDiff < iRange ){
                rollIdiff += rollDiff
            }
            if (pitchDiff > - iRange && pitchDiff < iRange) {
                pitchIdiff += pitchDiff
            }

        }

        let rollIcorrection = rollIdiff * rollPitchI
        let pitchIcorrection = pitchIdiff * rollPitchI

        rollIcorrection = Math.constrain(rollIcorrection, -iLimit, iLimit)     // Limit I (preventing it from growing out of proportions)
        pitchIcorrection = Math.constrain(pitchIcorrection, -iLimit, iLimit)

     
        rollCorrection = rollDiff * rollPitchP + rollIcorrection + rollDdiff * rollPitchD
        pitchCorrection = pitchDiff * rollPitchP + pitchIcorrection + pitchDdiff * rollPitchD
        yawCorrection = yawDiff * yawP + yawDdiff * yawD
        yawCorrection = Math.constrain(yawCorrection, -yawLimit, yawLimit)
        throttleScaled = throttle * 2.55

        motorA = Math.round(throttleScaled + rollCorrection + pitchCorrection + yawCorrection)
        motorB = Math.round(throttleScaled + rollCorrection - pitchCorrection - yawCorrection)
        motorC = Math.round(throttleScaled - rollCorrection + pitchCorrection - yawCorrection)
        motorD = Math.round(throttleScaled - rollCorrection - pitchCorrection + yawCorrection)
        motorA = Math.constrain(motorA, 0, 255)
        motorB = Math.constrain(motorB, 0, 255)
        motorC = Math.constrain(motorC, 0, 255)
        motorD = Math.constrain(motorD, 0, 255)
    }


    /**
     * Frame rate of pid loop
     */
    //% block

    export function fps() {

        return Math.round(1000 / looptime)
    }



    export function PCA_ReadMode1() {
        pins.i2cWriteNumber(
            PCA_REG_SLAVEADR,
            PCA_REG_MODE1,
            NumberFormat.UInt8BE,
            true
        )
        return pins.i2cReadNumber(PCA_REG_SLAVEADR, NumberFormat.UInt8BE, false)
    }


    let gyroReturnId = 0
    let mcReturnId = 0
    let throttleScaled = 0
    let yawCorrection = 0
    let pitchCorrection = 0
    let rollCorrection = 0
    let lastYawDiff = 0
    let pitchDdiff = 0
    let pitchDiff = 0
    let rollDiff = 0
    let accRollOffset = 0  //  Calibration offset of the Roll
    let accPitchOffset = 0  //  Calibration offset of the Pitch
    let oldTime = 0
    let yawIdiff = 0
    let yawDiff = 0
    let rollDdiff = 0
    let lastPitchDiff = 0
    let lastRollDiff = 0
    let pitchIdiff = 0
    let rollIdiff = 0
    let yawDdiff = 0
    let gyroZcalibration = 0
    let gyroZ = 0
    let gyroZdelta = 0
    let gyroXcalibration = 0
    let gyroX = 0
    let gyroXdelta = 0
    let looptime = 0
    let gyroYcalibration = 0
    let gyroY = 0
    let gyroYdelta = 0
    let accY = 0
    let accRoll = 0
    let accZ = 0
    let accX = 0
    let accPitch = 0
    let BATTERY_FACTOR = 5.94


    let PCA_REG_LEDUOT = 8
    let PCA_REG_SLAVEADR = 98
    let PCA_REG_MODE1 = 0
    let PCA_REG_MODE2 = 1
    let PCA_pwm0 = 2
    let PCA_pwm1 = 3
    let PCA_pwm2 = 4
    let PCA_pwm3 = 5
    let BARO_return = 0
    let PCA_REG_MODE2_CONFIG = 5        // Non-inverted: Open Drain: = %00001(1), Totem: = %00101(5), Inverted: Totem = %10101(21), Open drain: = %10001(17)

    let IMU_REG_CONFIG = 1          // 0x6b
    let IMU_PWR_MGMT_1 = 107        // 0x6b
    let IMU_WHO_AM_I = 117              // 0x68
    let IMU_SIGNAL_PATH_RESET = 105 // 0x6a
    let IMU_USER_CTRL = 106
    let IMU_ACCEL_CONFIG_2 = 29
    let IMU_REG_ADDRESS = 104
    let BARO_REG_SLAVEADR = 99





}

