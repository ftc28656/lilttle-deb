package org.firstinspires.ftc.teamcode.util

open class ButtonReader(val buttonValue : () -> Boolean) {
    private var lastButtonValue = buttonValue()

    val wasJustPressed : Boolean
        get() {
            val currentButtonValue = buttonValue()
            val justPressed = !lastButtonValue && currentButtonValue
            lastButtonValue = currentButtonValue
            return justPressed
        }
    val wasJustReleased : Boolean
        get() {
            val currentButtonValue = buttonValue()
            val justReleased = lastButtonValue && !currentButtonValue
            lastButtonValue = currentButtonValue
            return justReleased
        }
    val isPressed : Boolean
        get() = buttonValue()
    val isReleased : Boolean
        get() = !buttonValue()
}