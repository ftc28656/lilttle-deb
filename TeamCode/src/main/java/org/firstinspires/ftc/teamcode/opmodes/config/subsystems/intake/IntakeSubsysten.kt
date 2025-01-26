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
    private val intakeTimer = Timer()

    var state = IntakeStates.STOPPED
        set(value) {
            if(field != value)
                intakeTimer.resetTimer()

            field = value
            when(state) {
                IntakeStates.INTAKING -> intakeServo.position = LittleDebbie.intake.speeds.intake
                IntakeStates.OUTTAKING -> intakeServo.position = LittleDebbie.intake.speeds.outtake
                IntakeStates.STOPPED ->  intakeServo.position = LittleDebbie.intake.speeds.stop
                IntakeStates.HOLDING_ELEMENT -> intakeServo.position = LittleDebbie.intake.speeds.hold
            }

        }
    var element = IntakeElement.NONE
        private set;
    var distance = 0.0
        private set;
    var hsv = HSV(0.0F, 0.0F, 0.0F)
        private set;

    fun init() {
        intakeServo = hardwareMap.get<ServoImplEx>(ServoImplEx::class.java, "intake")
        colorSensor = hardwareMap.get(NormalizedColorSensor::class.java, "sensor_color")
        distanceSensor = (colorSensor as DistanceSensor)
        intakeTimer.resetTimer()
    }

    fun update() {
        hsv = colorSensor.normalizedColors.toHSV()
        distance = distanceSensor.getDistance(DistanceUnit.CM)

        // update the element that we are holding
        val prevElement = element
        element = if (distance < LittleDebbie.intake.distanceThreshold)
            when(hsv.hue) {
                in LittleDebbie.intake.colorLimits.red.minHue..LittleDebbie.intake.colorLimits.red.maxHue -> IntakeElement.RED
                in LittleDebbie.intake.colorLimits.yellow.minHue..LittleDebbie.intake.colorLimits.yellow.maxHue -> IntakeElement.YELLOW
                in LittleDebbie.intake.colorLimits.blue.minHue..LittleDebbie.intake.colorLimits.blue.maxHue -> IntakeElement.BLUE
                else -> IntakeElement.UNKNOWN
            }
        else
            IntakeElement.NONE

        // the micro seems to timeout after 5 seconds.  This just tracks that and mokes it official so that we.
        if(intakeTimer.elapsedTimeSeconds > LittleDebbie.intake.intakeTimeout)
            state = IntakeStates.STOPPED

        // now if we are moving from not holding an element to holding an element, move the intake to the hold position to hold the element
        // if we are moving from holding an element to not holding an element, stop the intake
//        if(prevElement == IntakeElement.NONE && element != IntakeElement.NONE)
//            state = IntakeStates.HOLDING_ELEMENT
//        else if(prevElement != IntakeElement.NONE && element == IntakeElement.NONE)
//            state = IntakeStates.STOPPED

    }

}