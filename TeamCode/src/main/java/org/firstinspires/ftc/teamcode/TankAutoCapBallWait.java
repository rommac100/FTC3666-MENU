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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTank;


/*

This Autonomous uses a angle that is placed done by the driver, then drives forward, and fires two projectiles, and knocks off Cap ball, and attempt to park.
 */

@Autonomous(name="Tank: AutotomousCapBallWait", group="Tank")
public class TankAutoCapBallWait extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTank robot = new HardwareTank();
    private ElapsedTime runtime = new ElapsedTime();

    public int distance(double dis) {
        return (int) (dis * robot.ticksPerInch);
    }


    //Enumeration for Direction choices
    public enum DIRECTION {
        FORWARD(+0.3), REVERSE(.25), Clockwise(.25), Counter_Clockwise(-.25);
        public final double value;

        DIRECTION(double value) {
            this.value = value;
        }
    }

    private int fEncoder = 0;
    private int fLastEncoder = 0;

    private long fVelocityTime = 0;
    private long fLastVelocityTime = 0;

    private double motorOut = 0.0;
    private double fTarget = 7.5e-7;
    private double fVelocity = 0.0;
    private double fError = 0.0;
    private double fLastError = 0.0;
    private double tolerance = 0.5e-7;


    //Our Drive forward method. Uses a declared Enumeration for Direction choices.
    public void drive(DIRECTION direction, int ticks) {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor.setTargetPosition(ticks);
        robot.rightMotor.setTargetPosition(ticks);
        double timeTemp = runtime.seconds() + 10;
        switch (direction) {
            case FORWARD:
                robot.leftMotor.setPower(DIRECTION.FORWARD.value);
                robot.rightMotor.setPower(DIRECTION.FORWARD.value);
                while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive()) {

                }
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);

                robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case REVERSE:
                robot.leftMotor.setTargetPosition(-ticks);
                robot.rightMotor.setTargetPosition(-ticks);

                robot.leftMotor.setPower(DIRECTION.REVERSE.value);
                robot.rightMotor.setPower(DIRECTION.REVERSE.value);
                while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive()) {
                    telemetry.addData("motorLeft Pos", robot.leftMotor.getCurrentPosition());
                    telemetry.update();
                }

                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);

                robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
        }
    }


    //Flywheel shooter method, has the passed in parameter of the duration it should be running.
    public void flyWheelShooter(double duration) {

        duration += runtime.seconds();
        while (runtime.seconds() < duration && opModeIsActive()) {
                bangBang();
            sleep(500);
                robot.spin1Motor.setPower(.42);
                bangBang();
            }

        setFPower(0);
        robot.spin1Motor.setPower(0);

    }

    private void setFPower(double power) {
        robot.flyWheelMotor1.setPower(power);
        robot.flyWheelMotor2.setPower(power);
    }

//Our compensation algorithm of the flywheel power, by adjusting the power of the flywheel based upon their velocity (by keeping track of encoder ticks).
    public void bangBang() {
        fVelocityTime = System.nanoTime();
        fEncoder = robot.flyWheelMotor1.getCurrentPosition();
        fVelocity = (double) (fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);
        if (fVelocity >= (fTarget + tolerance)) {
            setFPower(robot.minBangValue);
        } else if (fVelocity < (fTarget - tolerance)) {
            setFPower(robot.maxBangValue);
        }

        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;
    }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.addData("Alliance Colour", "Red or Blue");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        runtime.reset();
        if (opModeIsActive()) {

            while (runtime.seconds() < 15 && opModeIsActive())
            {}
            drive(DIRECTION.FORWARD, distance(42));
            sleep(1000);
            flyWheelShooter(6);
            drive(DIRECTION.FORWARD, distance(38));


            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            //sleep(1000);
        }
    }
}
