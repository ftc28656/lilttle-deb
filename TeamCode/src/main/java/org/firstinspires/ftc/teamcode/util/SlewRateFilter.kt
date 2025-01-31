package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.opmodes.teleop.SlewRateLimits
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer

class SlewRateFilter(private val slewRates : SlewRateLimits) {
    private var lastValue = 0.0
    private val updateTimer = Timer()

    fun filter(value: Double): Double {
        val deltaTime = updateTimer.elapsedTimeSeconds
        val delta = value - lastValue
        val maxPositiveDelta = slewRates.accel * deltaTime
        val maxNegativeDelta = slewRates.decel * deltaTime
        val clampedDelta = delta.coerceIn(-maxNegativeDelta, maxPositiveDelta)
        lastValue += clampedDelta
        updateTimer.resetTimer()
        return lastValue
    }
}