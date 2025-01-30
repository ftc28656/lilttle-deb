package org.firstinspires.ftc.teamcode.util

class TriggerReader(val trigger : () -> Float, val threshhold : Double = 0.5) : ButtonReader({ trigger() > threshhold}) {

}