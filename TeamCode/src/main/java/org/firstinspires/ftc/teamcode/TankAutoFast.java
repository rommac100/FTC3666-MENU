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

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTank;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


/*

This Autonomous uses a angle that is placed done by the driver, then drives forward, and fires two projectiles, and knocks off Cap ball, and attempt to park.
 */

@Autonomous(name="Tank: AutoFast", group="Tank")
public class TankAutoFast extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTank robot = new HardwareTank();
    private ElapsedTime runtime = new ElapsedTime();

    public int distance(double dis) {
        return (int) (dis * robot.ticksPerInch);
    }

    private String particlePref;
    private String beaconPref;
    private String capBallPref;
    private String parkingPref;
    private String alliance;

    //Enumeration for Direction choices
    public enum DIRECTION {
        FORWARD(+0.3), REVERSE(+.25), Clockwise(.25), Counter_Clockwise(-.25), Wiggle(.3);
        public final double value;

        DIRECTION(double value) {
            this.value = value;
        }
    }

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

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
    public void drive(DIRECTION direction, int ticks, double cutOffTime) {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double timeTemp = runtime.seconds() + cutOffTime;
        switch (direction) {
            case FORWARD:
                robot.leftMotor.setTargetPosition(ticks);
                robot.rightMotor.setTargetPosition(ticks);

                robot.leftMotor.setPower(DIRECTION.FORWARD.value);
                robot.rightMotor.setPower(DIRECTION.FORWARD.value);
                while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive() && runtime.seconds()<timeTemp) {
                    telemetry.addData("motorLeft Pos", robot.leftMotor.getCurrentPosition());
                    telemetry.update();
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
                while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive()&& runtime.seconds() < timeTemp) {
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

        robot.flyWheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duration += runtime.seconds();
        while (runtime.seconds() < duration && opModeIsActive()) {
                bangBang();
            sleep(1000);
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
        setFPower(robot.minBangValue+.15);
    } else if (fVelocity < (fTarget - tolerance)) {
        setFPower(robot.maxBangValue+.15);
    }

    fLastEncoder = fEncoder;
    fLastVelocityTime = fVelocityTime;
}


    public boolean turningDriveBoolean(double power, int angle, float angleDesired)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean temp = true;

        if (angle < 0)
        {
            robot.leftDrivePower = -power;
            robot.rightDrivePower = power;

            if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < angleDesired)
            {
                robot.leftDrivePower = 0;
                robot.rightDrivePower = 0;
                temp = false;

            }
            robot.leftMotor.setPower(robot.leftDrivePower);
            robot.rightMotor.setPower(robot.rightDrivePower);
        }
        else if (angle > 0)
        {
            robot.leftDrivePower = power;
            robot.rightDrivePower = -power;

            if (AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle) > angleDesired)
            {
                robot.leftDrivePower = 0;
                robot.rightDrivePower =0;
                temp = false;
            }
            robot.leftMotor.setPower(robot.leftDrivePower);
            robot.rightMotor.setPower(robot.rightDrivePower);
        }
        return temp;
    }


    public void turningDrive(double power, int angle)
    {
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        float angleDesired = AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle)+angle;

        if (angle < 0) {
            while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < angleDesired && opModeIsActive()) {
                robot.leftDrivePower = power;
                robot.rightDrivePower = -power;

                if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < angleDesired) {
                    robot.leftDrivePower = 0;
                    robot.rightDrivePower = 0;
                    robot.leftMotor.setPower(robot.leftDrivePower);
                    robot.rightMotor.setPower(robot.rightDrivePower);
                    break;
                }

                robot.leftMotor.setPower(robot.leftDrivePower);
                robot.rightMotor.setPower(robot.rightDrivePower);

                telemetry.update();

            }
            robot.leftMotor.setPower(robot.leftDrivePower);
            robot.rightMotor.setPower(robot.rightDrivePower);
        }

        else {
            while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > angleDesired && opModeIsActive()) {
                robot.leftDrivePower = -power;
                robot.rightDrivePower = power;

                if (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > angleDesired) {
                    robot.leftDrivePower = 0;
                    robot.rightDrivePower = 0;
                    robot.leftMotor.setPower(robot.leftDrivePower);
                    robot.rightMotor.setPower(robot.rightDrivePower);
                    break;
                }
                robot.leftMotor.setPower(robot.leftDrivePower);
                robot.rightMotor.setPower(robot.rightDrivePower);
                telemetry.update();


            }
            robot.leftMotor.setPower(robot.leftDrivePower);
            robot.rightMotor.setPower(robot.rightDrivePower);
        }
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initalize()
    {
        robot.init(hardwareMap);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;




    }
    private void getAutonomousPrefs()
    {
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        particlePref = preferences.getString("How Many Particles Should We Shoot?", "");
        beaconPref = preferences.getString("Which beacons should we activate?", "");
        capBallPref = preferences.getString("Should we bump the cap ball off the center vortex?", "");
        parkingPref = preferences.getString("Where should we park?", "");
        alliance = preferences.getString("Which alliance are we on?", "");
        telemetry.addData("Status", "Ready to run");
        telemetry.addData("Alliance Colour", "Red or Blue");
        telemetry.addLine("Particles: " + particlePref);
        telemetry.addLine("Beacons: " + beaconPref);
        telemetry.addLine("Cap Ball: " + capBallPref);
        telemetry.addLine("Parking: " + parkingPref);
        telemetry.addLine("Alliance: " + alliance);
        telemetry.update();
    }

    @Override
    public void runOpMode()
    {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        initalize();
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        getAutonomousPrefs();
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        runtime.reset();
        if (opModeIsActive()) {

            //drive(DIRECTION.REVERSE, distance(20));
            //flyWheelShooter(6);

            turnWithoutEncoders(DIRECTION.Counter_Clockwise);
            drive(DIRECTION.REVERSE, distance(4), 1.5);
            drive(DIRECTION.FORWARD, distance(4),1.5);
            drive(DIRECTION.FORWARD, distance(22),8);
            flyWheelShooter(4);
            drive(DIRECTION.REVERSE, distance(16),6);
            driveWiggle(robot.ticksPerInch/2, robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());
            checkColour(0);
            drive(DIRECTION.FORWARD,distance(25), 9.375);



            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

    public void turnWithoutEncoders(DIRECTION direction)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        double leftPowerInvert = -1;
        double rightPowerInvert = -1;

        if (alliance.equals("Red Alliance"))
        {
            leftPowerInvert = -.505;
            rightPowerInvert = -.8;
        }
        else if (alliance.equals("Blue Alliance"))
        {
            leftPowerInvert = -.8;
            rightPowerInvert = -.505;
        }

        if (direction == DIRECTION.Counter_Clockwise)
        {
            double tempTime = runtime.seconds() + 2.67;

            robot.leftMotor.setPower(leftPowerInvert);
            robot.rightMotor.setPower(rightPowerInvert);
            while (runtime.seconds() <tempTime && opModeIsActive())
            {


            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else
        {

        }
    }

    //used for caping the beacon
    public void driveWiggle(double ticks, int currentLeftTicks, int rightCurrentTicks)
    {
        Range.clip(ticks, robot.ticksPerInch*.1, robot.ticksPerInch);

        robot.rightMotor.setTargetPosition((int)(ticks+rightCurrentTicks));
        robot.leftMotor.setTargetPosition((int)(-ticks+currentLeftTicks));

        robot.driveMotors(DIRECTION.Wiggle.value, DIRECTION.Wiggle.value);

        while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive())
        {

        }

        robot.driveMotors(0, 0);

        robot.rightMotor.setTargetPosition((int)(robot.rightMotor.getCurrentPosition()-ticks));
        robot.leftMotor.setTargetPosition((int)(robot.leftMotor.getCurrentPosition()+ticks));

        robot.driveMotors(DIRECTION.Wiggle.value, DIRECTION.Wiggle.value);

        while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive())
        {

        }
        robot.driveMotors(0, 0);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
    private void checkColour(double attemptsMade)
    {
        //Values that we care from the colour sensor


        int[] colourSensor1 = robot.muxColor.getCRGB(robot.ports[0]);
        int[] colourSensor2 = robot.muxColor.getCRGB(robot.ports[1]);

        //this is for red alliance
        if (attemptsMade < 2)
        {
            if (colourSensor1[3] > colourSensor1[1] && alliance.equals("Red Alliance"))
            {
                double firstHit = runtime.seconds();
                if (attemptsMade != 0) {
                    while (runtime.seconds() < firstHit + 5.1) ;

                }
                attemptsMade += 1;
                //driveWiggle(robot.ticksPerInch/2, robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());
                drive(DIRECTION.REVERSE, distance(17), 6.375);

                drive(DIRECTION.FORWARD, distance(5),1.5625);
                checkColour(attemptsMade);
            }
            else if (colourSensor2[1] > colourSensor2[3] && alliance.equals("Blue Alliance"))
            {
                attemptsMade += 1;
                //driveWiggle(robot.ticksPerInch/2, robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());
                drive(DIRECTION.REVERSE, distance(17),6.375);

                drive(DIRECTION.FORWARD, distance(5),1.5625);
                checkColour(attemptsMade);
            }
            else
            {

            }

        }

    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

