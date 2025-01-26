package org.firstinspires.ftc.teamcode.opmodes.config.subsystems.rearlight

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer

class RearLightSubsystem(val hardwareMap: HardwareMap) {
    val rearLight= hardwareMap.get<Servo>(Servo::class.java, "rearLight")
    val flashTimer = Timer()

    // teken from gobilda page https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
    val blueColorPosition = 0.611
    val redColorPosition = 0.279
    val yellowColorPosition = 0.388
    val greenColorPosition = 0.5
    val offColorPosition = 0.1
    val whiteColorPosition = 0.9
    val purpleColorPosition = 0.72

    var state = RearLightStates.EMPTY_AND_OFF
        set(value) {
            field = value
            updateRearLightState()
        }

    fun init() {
        flashTimer.resetTimer()
        state = RearLightStates.INITIALIZEDANDREADY
    }
    fun update() {
        updateRearLightState()
    }
    private fun updateRearLightState() {
        val flashPeriod = 0.5 // seconds
        // take the elapsed time in secs and modulo 2.  If less than 1, set to red, else set to off
        val flashState = flashTimer.elapsedTimeSeconds % flashPeriod < (flashPeriod/2.0)

        when(state) {
            // flash green, then white
            RearLightStates.INITIALIZING -> {
                if(flashState)
                    rearLight.position = greenColorPosition
                else
                    rearLight.position = whiteColorPosition
            }
            RearLightStates.INITIALIZEDANDREADY -> rearLight.position = greenColorPosition
            RearLightStates.HOLDING_RED -> rearLight.position = redColorPosition
            RearLightStates.HOLDING_BLUE -> rearLight.position = blueColorPosition
            RearLightStates.HOLDING_YELLOW -> rearLight.position = yellowColorPosition
            RearLightStates.HOLDING_UNKOWN -> rearLight.position = greenColorPosition
            RearLightStates.INTAKING_REDORYELLOW -> {
                // flash red, then yellow
                if(flashState)
                    rearLight.position = redColorPosition
                else
                    rearLight.position = yellowColorPosition
            }
            RearLightStates.INTAKING_BLUEORYELLOW -> {
                // flash blue, then yellow
                if(flashState)
                    rearLight.position = blueColorPosition
                else
                    rearLight.position = yellowColorPosition
            }
            RearLightStates.INTAKING -> {
                // flash green, then white
                if(flashState)
                    rearLight.position = purpleColorPosition
                else
                    rearLight.position = whiteColorPosition
            }
            RearLightStates.OUTTAKING -> {
                // flash green, then white
                if(flashState)
                    rearLight.position = greenColorPosition
                else
                    rearLight.position = whiteColorPosition
            }
            RearLightStates.EMPTY_AND_OFF -> rearLight.position = offColorPosition
        }
    }
}