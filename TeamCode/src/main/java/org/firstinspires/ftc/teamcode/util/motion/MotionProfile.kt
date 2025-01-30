package org.firstinspires.ftc.teamcode.util.motion

import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer

class MotionProfile(val start : MotionState, val target : MotionState, val steps : List<MotionProfileStep>) {
    private val profileTimer = Timer()
    private var lastStepIndex = 0

    fun start() {
        profileTimer.resetTimer()
        lastStepIndex = 0
    }
    fun next() : MotionState {
        val currentProfileTimeMs = profileTimer.elapsedTime

        // Move the lastStepIndex forward while the step time is <= current time.
        // Using '<=' means we skip all steps up to and including the current time,
        // so the returned step's timeMs is the *first* strictly greater than current time.
        while (lastStepIndex < steps.size && steps[lastStepIndex].timeMs <= currentProfileTimeMs) {
            lastStepIndex++
        }

        // If we've gone beyond the end, clamp to the last step.
        if (lastStepIndex >= steps.size) {
            lastStepIndex = steps.size - 1
        }

        return steps[lastStepIndex].motionState

    }
}