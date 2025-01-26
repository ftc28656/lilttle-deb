package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer

class SlewRateFilter(private val maxSlewRateProvider : () -> Double) {
    constructor(maxSlewRate: Double) : this({ maxSlewRate })
    private var lastValue = 0.0
    private val updateTimer = Timer()

    fun filter(value: Double): Double {
        val deltaTime = updateTimer.elapsedTimeSeconds
        val delta = value - lastValue
        val maxDelta = maxSlewRateProvider() * deltaTime
        val clampedDelta = delta.coerceIn(-maxDelta, maxDelta)
        lastValue += clampedDelta
        updateTimer.resetTimer()
        return lastValue
    }
}