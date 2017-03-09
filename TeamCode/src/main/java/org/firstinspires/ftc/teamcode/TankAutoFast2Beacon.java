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

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTank;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Locale;


/*
   This auto goes for the one or two beacons depending on the settings selected, and will shoot two projectiles into the center vortex, essentially a 90 point autonomous.
 */

@Autonomous(name="Tank: AutoFastTesting", group="Tank")
public class TankAutoFast2Beacon extends LinearOpMode {

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
        FORWARD(+0.45), REVERSE(+.45), Clockwise(.4), Counter_Clockwise(.25), Wiggle(.3), Flick(-.3), Fire(.35), Distance(-.4);
        public final double value;

        DIRECTION(double value) {
            this.value = value;
        }
    }

    BNO055IMU imu;

    boolean firstBeacon = false;

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

    private double targetVoltage = 13;
    private double voltage;

    private double targetVoltageTurning = 12.5;
    private double time1 = 0;
    private double time2 = 0;



    /*
    Our main drive method. It uses a enumeration for the selection of direction and power. And a long switch structure to decide what happens afterwards.
    Our flicker's system is part of this this particular method. The flickers are deployed before we hit the wall in order to attempt for the irregularities of our arced turn.
     */
    public void driveOverHaul(DIRECTION direction, int ticks, double cutOffTime, double fireTime) {

        int currentTicksLeft = robot.leftMotor.getCurrentPosition();
        int currentTicksRight = robot.rightMotor.getCurrentPosition();
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        double timeTemp = runtime.seconds() + cutOffTime;
        switch (direction) {
            case FORWARD:
                robot.leftMotor.setTargetPosition(ticks+currentTicksLeft);
                robot.rightMotor.setTargetPosition(ticks+currentTicksRight);

                robot.leftMotor.setPower(DIRECTION.FORWARD.value);
                robot.rightMotor.setPower(DIRECTION.FORWARD.value);
                double fireTimeTemp = runtime.seconds()+fireTime;
                while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive() && runtime.seconds()<timeTemp) {
                    telemetry.addData("motorLeft Pos", robot.leftMotor.getCurrentPosition());
                    telemetry.update();
                    if (runtime.seconds() <fireTime) {
                        voltageProportional();
                        sleep(200);
                        robot.spin1Motor.setPower(.42);

                    }
                    idle();
                }
                setFPower(0);
                robot.spin1Motor.setPower(0);
                robot.spin1Motor.setPower(0);
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                break;

            case Fire:
                robot.leftMotor.setTargetPosition(ticks+currentTicksLeft);
                robot.rightMotor.setTargetPosition(ticks+currentTicksRight);

                robot.leftMotor.setPower(DIRECTION.FORWARD.value);
                robot.rightMotor.setPower(DIRECTION.FORWARD.value);
                while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive() && runtime.seconds()<timeTemp) {
                    telemetry.addData("motorLeft Pos", robot.leftMotor.getCurrentPosition());
                    telemetry.update();
                    if (runtime.seconds() <fireTime) {
                        voltageProportional();
                        sleep(200);
                        robot.spin1Motor.setPower(.42);

                    }
                }
                setFPower(0);
                robot.spin1Motor.setPower(0);
                robot.spin1Motor.setPower(0);
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                 break;
            case REVERSE:
                robot.leftMotor.setTargetPosition(-ticks+currentTicksLeft);
                robot.rightMotor.setTargetPosition(-ticks+currentTicksRight);

                robot.leftMotor.setPower(DIRECTION.REVERSE.value);
                robot.rightMotor.setPower(DIRECTION.REVERSE.value);
                while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive() && runtime.seconds() < timeTemp) {
                    telemetry.addData("motorLeft Pos", robot.leftMotor.getCurrentPosition());
                    telemetry.update();
                    idle();
                    if (runtime.seconds() <fireTime) {
                        if (runtime.seconds() <fireTime-2)
                        {
                            voltageProportional();
                        }
                        else
                        {
                            robot.spin1Motor.setPower(.1);
                        }

                    }
                }
                setFPower(0);
                robot.spin1Motor.setPower(0);

                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                break;
            case Flick:
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.flickServoOut();
                robot.driveMotors(direction.value, direction.value);

                while (opModeIsActive() && !robot.limit1.returnState() && robot.limit2.returnState() && runtime.seconds() <timeTemp)
                {
                    telemetry.addData("limit1", robot.limit1.returnState());
                    telemetry.addData("limit2", robot.limit2.returnState());
                    telemetry.update();
                }
                driveOverHaul(DIRECTION.FORWARD,distance(2),2,0);
                robot.flickServoIn();
                driveOverHaul(DIRECTION.REVERSE,distance(5),3,0);
                robot.driveMotors(0,0);
                break;
            case Distance:
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.driveMotors(direction.value, direction.value);

                while (opModeIsActive() && robot.ultraSonic.getDistance(DistanceUnit.INCH) > ticks)
                {
                    telemetry.addData("distance", robot.ultraSonic.getDistance(DistanceUnit.INCH));
                    idle();
                }
                robot.driveMotors(0,0);
        }
    }


    //Flywheel shooter method, has the passed in parameter of the duration it should be running.
    public void flyWheelShooter(double duration) {
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duration += runtime.seconds();
        while (runtime.seconds() < duration && opModeIsActive()) {
            voltageProportional();
            robot.spin1Motor.setPower(.42);
            voltageProportional();
        }

        setFPower(0);
        robot.spin1Motor.setPower(0);

    }
    public void voltageProportional()
    {
        voltage = robot.getBatteryVoltage();
        double kP = .15;
        double error = targetVoltage - voltage;
        motorOut = (error * kP) + .75;
        motorOut = Range.clip(motorOut, 0, 1);
        setFPower(motorOut);
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

    /*
    This simple method turns our robot based upon an angle and uses encoders.
     */

    public void turnEncoder(int angle)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int ticks = angle * distance(.15);
        if (angle > 0)
        {
            robot.leftMotor.setTargetPosition(ticks+robot.leftMotor.getCurrentPosition());
            robot.rightMotor.setTargetPosition(-ticks+robot.rightMotor.getCurrentPosition());
            robot.driveMotors(DIRECTION.Clockwise.value, DIRECTION.Clockwise.value);
            while (opModeIsActive() && robot.leftMotor.isBusy() && robot.rightMotor.isBusy())
            {

            }
            robot.driveMotors(0,0);
        }
        else if (angle< 0)
        {
            robot.leftMotor.setTargetPosition((ticks+robot.leftMotor.getCurrentPosition()));
            robot.rightMotor.setTargetPosition(-ticks+robot.rightMotor.getCurrentPosition());
            robot.driveMotors(DIRECTION.Counter_Clockwise.value,DIRECTION.Counter_Clockwise.value);

            while (opModeIsActive() && robot.leftMotor.isBusy() && robot.rightMotor.isBusy())
            {

            }
            robot.driveMotors(0,0);
        }
    }

    public void initalize()
    {
        robot.init(hardwareMap);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        getAutonomousPrefs();
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

    /*
    A form of battery compensation for our robot, we essentially spin our flywheel motors in order
    to do a form of battery voltage regulation by reducing the power of the overall system.
     */
    private void adjustVoltage()
    {
        double currentFlyPower = 0;
       if (robot.getBatteryVoltage() > 13 && runtime.seconds() < 3)
       {
           currentFlyPower++;
           setFPower(currentFlyPower);
           adjustVoltage();
       }
    }

    //selects whether or not to go fro second beacon if the time amount is correct.

    public void alternativeRoutes()
    {
        if (runtime.seconds() < 7 && firstBeacon)
        {
            driveOverHaul(DIRECTION.FORWARD,distance(42), 8,0);
        }
        else if (runtime.seconds() > 12 && firstBeacon) {

            //This form of driving turns 92 ish degrees and then drives towards the second beacon, and turns again to face the beacon and attempts to capture it.
            driveOverHaul(DIRECTION.FORWARD, distance(7), 2, 0);
            turnEncoder(-92);
            adjustVoltage();
            driveOverHaul(DIRECTION.REVERSE, distance(48), 15, 0);
            turnEncoder(92);
            driveOverHaul(DIRECTION.REVERSE, distance(16), 9, 0);
            checkColour(0);
               /* An alternate type of turn in order to get the second beacon. It is uses a variable arc instead of a turn and drive method.
                driveOverHaul(DIRECTION.FORWARD, distance(7), 2, 0);
                turnEncoder(180);
                adjustVoltage();
                arcedTurn(DIRECTION.Counter_Clockwise, .35, .75, 3.2, true, robot.getBatteryVoltage());
                driveOverHaul(DIRECTION.REVERSE, distance(16), 9, 0);
                checkColour(0);
                */
        }
    }


    /*
    In this function we do our arced turn to the first beacon which we decided was the most effective solution to the problem of time and accuracy for our robot specifically.
    Then it attempts to drive foward from the beacon and fire while doing so, then it checks whether or not the first press of the beacon was correct.
     */
    public void beacon1(){
        robot.flickServoOut();
        arcedTurn(DIRECTION.Counter_Clockwise, .505,.8,2.65, false, robot.getBatteryVoltage());
        driveOverHaul(DIRECTION.Flick,0,3,0);
        time1 = runtime.seconds();
        driveOverHaul(DIRECTION.FORWARD, distance(4),1.5,0);
        driveOverHaul(DIRECTION.FORWARD, distance(4),1.5,0);
        driveOverHaul(DIRECTION.Fire, distance(18),10,10);
        driveOverHaul(DIRECTION.REVERSE, distance(23),8,0);
        time2 = runtime.seconds();
        if (time1-time2 < 5)
        {
            sleep((long)(5-(time1-time2)*1000));
        }
        checkColour(0);
        firstBeacon = true;
    }

    public double getHeading() {return angles.firstAngle;}

    @Override
    public void runOpMode()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        initalize();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        composeTelemetry();

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        runtime.reset();
        if (opModeIsActive()) {
            adjustVoltage();
           beacon1();
            alternativeRoutes();
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }


    /*
    This particular method comprises the entirety of our peculiar arcedTurning function of our robot.
    We use this in order to get the first beacon and as a backup for the second beacon.
    It uses the concept of time, and varying the speed of each drive train motor in order to get the correct arc.
    Also in this function we can vary the power of the motors in order to get a not constant arc but varied arc which is used for the second beacon if needed.
     */
    public void arcedTurn(DIRECTION direction, double leftPower, double rightPower, double cutOffTime, boolean variable, double currentBatteryVoltage)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        double leftPowerInvert = 0;
        double rightPowerInvert = 0;

        if (alliance.equals("Red Alliance"))
        {
            leftPowerInvert = -leftPower;
            rightPowerInvert = -rightPower;
        }
        else if (alliance.equals("Blue Alliance"))
        {
            leftPowerInvert = -rightPower;
            rightPowerInvert = -leftPower;
        }

        if (direction == DIRECTION.Counter_Clockwise)
        {
            double tempTime = runtime.seconds() + cutOffTime;

            robot.leftMotor.setPower(leftPowerInvert);
            robot.rightMotor.setPower(rightPowerInvert);

            double temptime = runtime.seconds();


            double powerLeft1 = 0;
            double powerRight1 = 0;

            double powerLeft2 = 0;
            double powerRight2 = 0;


            if (variable)
            {
                powerLeft1 = .2;
                powerRight1 = .95;

                powerLeft2 = .2;
                powerRight2 = .65;
            }
            while (runtime.seconds() <tempTime && opModeIsActive())
            {
                if (variable) {

                    if (runtime.seconds() < tempTime-2.7-temptime)
                    {
                        robot.driveMotors(powerLeft1,powerRight1);
                    }
                    else if (runtime.seconds() < tempTime-1.1-temptime) {
                        robot.driveMotors(powerLeft2, powerRight2);
                    }
                }


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


    /*
    This checkColour method is a recursion form of loop that continues checking the beacon if it is not the correct colour.
    It is necessary to have this form loop do to our beacon capturing being done with a flat plate.

    We utilize do colour sensors due to the arc of our turn being slightly different for each alliance.
     */
    private void checkColour(double attemptsMade)
    {
        //Values that we care from the colour sensors
        int[] colourSensor1 = robot.muxColor.getCRGB(robot.ports[0]);
        int[] colourSensor2 = robot.muxColor.getCRGB(robot.ports[1]);

        //this is for red alliance
        if (attemptsMade < 2)
        {
            if (colourSensor2[3] > colourSensor2[1] && alliance.equals("Red Alliance"))
            {
                double firstHit = runtime.seconds();
                if (attemptsMade != 0) {
                    while (runtime.seconds() < firstHit + 5.1) ;

                }
                attemptsMade += 1;
                driveOverHaul(DIRECTION.REVERSE, distance(17), 6.375,0);

                driveOverHaul(DIRECTION.FORWARD, distance(5),1.5625,0);
                checkColour(attemptsMade);
            }
            else if (colourSensor1[1] > colourSensor1[3] && alliance.equals("Blue Alliance"))
            {
                attemptsMade += 1;
                driveOverHaul(DIRECTION.REVERSE, distance(17),6.375,0);

                driveOverHaul(DIRECTION.FORWARD, distance(5),1.5625,0);
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

