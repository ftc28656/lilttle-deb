package org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake

import android.graphics.Color
import com.qualcomm.robotcore.hardware.NormalizedRGBA

data class HSV(val hue : Float, val saturation : Float, val value : Float)

fun NormalizedRGBA.toHSV(): HSV {
    val hsvValues = FloatArray(3)
    Color.colorToHSV(this.toColor(), hsvValues)
    return HSV(hsvValues[0], hsvValues[1], hsvValues[2])
}
