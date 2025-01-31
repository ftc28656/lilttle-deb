package org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake

import android.graphics.Color
import com.qualcomm.robotcore.hardware.NormalizedRGBA

data class HSV(val hue : Float, val saturation : Float, val value : Float){
    var isUnknown = false
        private set
    private constructor() : this(0.0F, 0.0F, 0.0F) {
        isUnknown =true
    }
    companion object {
        val unkown = HSV()
    }
    override fun toString(): String {
        if(isUnknown)
            return "HSV(unkown)"
        else
            return "HSV(hue=$hue, saturation=$saturation, value=$value)"
    }
}

fun NormalizedRGBA.toHSV(): HSV {
    val hsvValues = FloatArray(3)
    Color.colorToHSV(this.toColor(), hsvValues)
    return HSV(hsvValues[0], hsvValues[1], hsvValues[2])
}
