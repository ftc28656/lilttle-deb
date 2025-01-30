package org.firstinspires.ftc.teamcode.util.motion

import android.util.Log
import edu.ncssm.ftc.electricmayhem.core.util.epsilonEquals
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer
import kotlin.math.sign

class MotionProfileGenerator(val maxVelocity: Double, val maxAcceleration: Double, val timeStepMs : Long) {
    init {
        if (maxVelocity <= 0.0) throw IllegalArgumentException("Max velocity must be positive")
        if (maxAcceleration <= 0.0) throw IllegalArgumentException("Max acceleration must be positive")
        if (timeStepMs <= 0) throw IllegalArgumentException("Time step must be positive")
        if (maxAcceleration epsilonEquals 0.0) throw IllegalArgumentException("Max acceleration must be non-zero")
    }

    var currentState = MotionState(0.0,0.0)
        private set
    val deltaTimer = Timer()
    private var isFirstUpate = true

    fun init(init : Double, target : Double) : MotionProfile {
        return init(MotionState(init,0.0), MotionState(target, 0.0))
    }
    fun init(init : MotionState, targetState: MotionState) : MotionProfile {
        currentState = init
        return getNewMotionProfile(targetState)
    }
    fun update(x : Double) {
        if(isFirstUpate) {
            isFirstUpate = false
            currentState = MotionState(x,0.0)
        } else {
            val deltaT = deltaTimer.elapsedTimeSeconds
            val v = (x - currentState.x)/deltaT
            currentState = MotionState(x,v)
            deltaTimer.resetTimer()
        }
    }
    fun getNewMotionProfile(targetState: MotionState) : MotionProfile {
        val steps = generateSCurveMotionProfile(currentState, targetState)
        return MotionProfile(currentState, targetState, steps)
    }

    private fun generateSCurveMotionProfile(currentState: MotionState, targetState: MotionState): List<MotionProfileStep> {
        val motionProfileSteps = mutableListOf<MotionProfileStep>()
        val timeStep = timeStepMs / 1000.0
        var t = 0.0
        val direction = sign(targetState.x - currentState.x)
        var x = currentState.x
        var v = currentState.v
        var a = maxAcceleration * direction

        // Define the different phases of motion
        val accelTime = maxVelocity / maxAcceleration
        val coastTime = (targetState.x - currentState.x) / maxVelocity - accelTime

        Log.d("motion","current.x=${currentState.x} target.x=${targetState.x}")

        var phase = 1
        var reachedTarget = false
        var lastMotionState = currentState
        while (!reachedTarget) {
            when (phase) {
                1 -> { // Constant acceleration
                    if (t >= accelTime) phase++
                }
                2 -> { // Constant velocity
                    a = 0.0
                    if (t >= accelTime + coastTime) phase++
                }
                3 -> { // Constant deceleration
                    a = -direction * maxAcceleration
                    if (t >= accelTime + coastTime + accelTime) phase++
                }
                4 -> {
                    a = 0.0
                    v = 0.0
                }
            }

            v += a * timeStep
            x += v * timeStep

//            Log.d("motion","direction=$direction, x=$x, v=$v, a=$a, target.x=${targetState.x}")
            // this is the last step - align it to the target point no matter the timeStep
            if ((direction >= 0.0 && x >= targetState.x) || (direction < 0.0 && x <= targetState.x)) {
                reachedTarget = true
                x = targetState.x
                v = (x-lastMotionState.x)/timeStep
                a = (v-lastMotionState.v)/timeStep
            }

            lastMotionState = MotionState(x,v,a)
            motionProfileSteps.add(MotionProfileStep(t.toLong(), lastMotionState))
            t += timeStep
        }
        return motionProfileSteps
    }
}


