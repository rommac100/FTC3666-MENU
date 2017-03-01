/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTank;

import static java.lang.Thread.sleep;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Hardware Tank: SensorDignosis", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class SensorDiagnostic extends OpMode
{
    /* Declare OpMode members. */
    public ElapsedTime runtime = new ElapsedTime();
    HardwareTank robot = new HardwareTank();

    private double maxDriveTrain;
    private double maxIntakeSystem;
    private boolean direction = true; // true equals normal direction
    private boolean drift = true;
    private double halfSpeed = 1;       //current speed reduction coefficient.  1 at normal power.
    private double leftQ;
    private double centerQ;
    private double rightQ;
    private boolean limit1;
    private boolean limit2;
    double distance = 0;
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.spin1Motor.setPower(0);
        robot.spin2Motor.setPower(0);

        robot.flyWheelMotor1.setPower(0);
        robot.flyWheelMotor2.setPower(0);
        for (int i=0; i<robot.ports.length; i++) {
            int[] crgb = robot.muxColor.getCRGB(robot.ports[i]);

            telemetry.addLine("Sensor " + robot.ports[i]);
            telemetry.addData("CRGB", "%5d %5d %5d %5d",
                    crgb[0], crgb[1], crgb[2], crgb[3]);
        }
        distance = robot.ping.getDistanceCM(runtime);

        telemetry.addData("Distance", distance);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        leftQ  = robot.device.getAnalogInputVoltage(3);
        centerQ= robot.device.getAnalogInputVoltage(0);
        rightQ = robot.device.getAnalogInputVoltage(5);


        for (int i=0; i<robot.ports.length; i++) {
            int[] crgb = robot.muxColor.getCRGB(robot.ports[i]);

            telemetry.addLine("Sensor " + robot.ports[i]);
            telemetry.addData("CRGB", "%5d %5d %5d %5d",
                    crgb[0], crgb[1], crgb[2], crgb[3]);
        }

        telemetry.addData("Left", leftQ);
        telemetry.addData("Right", rightQ);
        telemetry.addData("Center", centerQ);

        telemetry.addData("Clear", robot.colourSensor.alpha());
        telemetry.addData("Red  ", robot.colourSensor.red());
        telemetry.addData("Green", robot.colourSensor.green());
        telemetry.addData("Blue ", robot.colourSensor.blue());
        telemetry.addData("Distance", distance);
        telemetry.addData("endStop1", robot.limit1.returnState());
        telemetry.addData("endStop2", robot.limit2.returnState());
        //robot.ping.turnOn();
    }

    @Override
    public void stop() {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.spin1Motor.setPower(0);
        robot.spin2Motor.setPower(0);

        robot.flyWheelMotor1.setPower(0);
        robot.flyWheelMotor2.setPower(0);
    }

}
