/* Copyright (c) 2017-2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes.test;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.rearlight.RearLightStates;
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.rearlight.RearLightSubsystem;

/*
 * This OpMode shows how to use a color sensor in a generic
 * way, regardless of which particular make or model of color sensor is used. The OpMode
 * assumes that the color sensor is configured with a name of "sensor_color".
 *
 * There will be some variation in the values measured depending on the specific sensor you are using.
 *
 * You can increase the gain (a multiplier to make the sensor report higher values) by holding down
 * the A button on the gamepad, and decrease the gain by holding down the B button on the gamepad.
 *
 * If the color sensor has a light which is controllable from software, you can use the X button on
 * the gamepad to toggle the light on and off. The REV sensors don't support this, but instead have
 * a physical switch on them to turn the light on and off, beginning with REV Color Sensor V2.
 *
 * If the color sensor also supports short-range distance measurements (usually via an infrared
 * proximity sensor), the reported distance will be written to telemetry. As of September 2020,
 * the only color sensors that support this are the ones from REV Robotics. These infrared proximity
 * sensor measurements are only useful at very small distances, and are sensitive to ambient light
 * and surface reflectivity. You should use a different sensor if you need precise distance measurements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: Color", group = "Sensor")
public class SensorColorDistanceTester extends LinearOpMode {

  /** The colorSensor field will contain a reference to our color sensor hardware object */
  NormalizedColorSensor colorSensor;
  RearLightSubsystem rearLightSubsystem;

  @Override public void runOpMode() {
    runSample(); // actually execute the sample
  }

  protected void runSample() {
    // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
    // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
    // can give very low values (depending on the lighting conditions), which only use a small part
    // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
    // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
    // colors will report at or near 1, and you won't be able to determine what color you are
    // actually looking at. For this reason, it's better to err on the side of a lower gain
    // (but always greater than  or equal to 1).
    float gain = 2;

    // Once per loop, we will update this hsvValues array. The first element (0) will contain the
    // hue, the second element (1) will contain the saturation, and the third element (2) will
    // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
    // for an explanation of HSV color.
    final float[] hsvValues = new float[3];

    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
    rearLightSubsystem = new RearLightSubsystem(hardwareMap);
    rearLightSubsystem.init();

    // Wait for the start button to be pressed.
    waitForStart();

    // Loop until we are asked to stop
    while (opModeIsActive()) {
      // Explain basic gain information via telemetry
      telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
      telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

      // Update the gain value if either of the A or B gamepad buttons is being held
      if (gamepad1.a) {
        // Only increase the gain by a small amount, since this loop will occur multiple times per second.
        gain += 0.005F;
      } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
        gain -= 0.005F;
      }

      // Show the gain value via telemetry
      telemetry.addData("Gain", gain);

      // Tell the sensor our desired gain value (normally you would do this during initialization,
      // not during the loop)
      colorSensor.setGain(gain);

      // Get the normalized colors from the sensor
      NormalizedRGBA colors = colorSensor.getNormalizedColors();

      /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
       * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
       * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
       * for an explanation of HSV color. */

      // Update the hsvValues array by passing it to Color.colorToHSV()
      Color.colorToHSV(colors.toColor(), hsvValues);

      float hue = hsvValues[0];
      double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
      double threshold = 2.5; //cm

      if(distance < threshold)
        if(hue < 40)
          rearLightSubsystem.setState(RearLightStates.HOLDING_RED);
        else if(hue <150)
          rearLightSubsystem.setState(RearLightStates.HOLDING_YELLOW);
        else
          rearLightSubsystem.setState(RearLightStates.HOLDING_BLUE);
      else
        rearLightSubsystem.setState(RearLightStates.EMPTY_AND_OFF);

      rearLightSubsystem.update();

//      telemetry.addLine()
//              .addData("Red", "%.3f", colors.red)
//              .addData("Green", "%.3f", colors.green)
//              .addData("Blue", "%.3f", colors.blue);
      telemetry.addLine()
              .addData("Hue", "%.3f", hsvValues[0])
              .addData("Saturation", "%.3f", hsvValues[1])
              .addData("Value", "%.3f", hsvValues[2]);
      telemetry.addData("Alpha", "%.3f", colors.alpha);
      telemetry.addData("Distance (cm)", "%.3f", distance);
      telemetry.addData("Rear Light State", rearLightSubsystem.getState().toString());
      telemetry.addData("Rear Light FlashTimer", rearLightSubsystem.getFlashTimer().getElapsedTimeSeconds());
      telemetry.update();

    }
  }
}
