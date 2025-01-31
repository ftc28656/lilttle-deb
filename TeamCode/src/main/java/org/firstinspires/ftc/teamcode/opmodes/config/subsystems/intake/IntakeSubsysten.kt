package org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.opmodes.config.LittleDebbie
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer

class IntakeSubsysten(val hardwareMap: HardwareMap) {
    private lateinit var intakeServo: ServoImplEx
    private lateinit var colorSensor: NormalizedColorSensor
    private lateinit var distanceSensor: DistanceSensor
    private val pauseTimer = Timer()
    private val autoStopTimer = Timer()
    var pausedState = IntakeStates.PAUSED
        private set
    var state = IntakeStates.STOPPED
        set(value) {
            if(field != value)
                pauseTimer.resetTimer()

            when(value) {
                IntakeStates.INTAKING -> intakeServo.position = LittleDebbie.intake.speeds.intake
                IntakeStates.OUTTAKING -> intakeServo.position = LittleDebbie.intake.speeds.outtake
                IntakeStates.STOPPED ->  intakeServo.position = LittleDebbie.intake.speeds.stop
                IntakeStates.HOLDING_ELEMENT -> intakeServo.position = LittleDebbie.intake.speeds.hold
                IntakeStates.PAUSED -> {
                    pausedState = state
                    intakeServo.position = LittleDebbie.intake.speeds.stop
                }
            }
            field = value
        }
    var element = IntakeElement.NONE
        private set
    var distance = 0.0
        private set
    var hsv = HSV(0.0F, 0.0F, 0.0F)
        private set
    val servoPosition
        get() = intakeServo.position

    fun init() {
        intakeServo = hardwareMap.get<ServoImplEx>(ServoImplEx::class.java, "intake")
        colorSensor = hardwareMap.get(NormalizedColorSensor::class.java, "sensor_color")
        distanceSensor = (colorSensor as DistanceSensor)
        pauseTimer.resetTimer()
    }

    fun update() {
        distance = distanceSensor.getDistance(DistanceUnit.CM)

        // update the element that we are holding
        var previousElement = element
        element = if (distance < LittleDebbie.intake.distanceThreshold){
                hsv = colorSensor.normalizedColors.toHSV()
                when(hsv.hue) {
                    in LittleDebbie.intake.colorLimits.red.minHue..LittleDebbie.intake.colorLimits.red.maxHue -> IntakeElement.RED
                    in LittleDebbie.intake.colorLimits.yellow.minHue..LittleDebbie.intake.colorLimits.yellow.maxHue -> IntakeElement.YELLOW
                    in LittleDebbie.intake.colorLimits.blue.minHue..LittleDebbie.intake.colorLimits.blue.maxHue -> IntakeElement.BLUE
                    else -> IntakeElement.UNKNOWN
                }
            }
        else {
            hsv = HSV.unkown
            IntakeElement.NONE
        }

        // the micro seems to timeout after 5 seconds.  This just tracks that and mokes it official so that we.
        when(state) {
            IntakeStates.INTAKING,
            IntakeStates.OUTTAKING -> {
                if(pauseTimer.elapsedTimeSeconds > LittleDebbie.intake.crTimeout)
                    state = IntakeStates.PAUSED
            }
            IntakeStates.PAUSED -> {
                if(pauseTimer.elapsedTimeSeconds > LittleDebbie.intake.crPause)
                    state = pausedState
            }
            else -> { }
        }

        // when we no longer have an element, stop after a short delay to allow full intake/outtake
        if(previousElement != element)
            autoStopTimer.resetTimer()

        if( ((state == IntakeStates.OUTTAKING && element == IntakeElement.NONE)
            || (state == IntakeStates.INTAKING && element != IntakeElement.NONE))
            && autoStopTimer.elapsedTimeSeconds > LittleDebbie.intake.autoStopDelay)
            state = IntakeStates.STOPPED

    }

}