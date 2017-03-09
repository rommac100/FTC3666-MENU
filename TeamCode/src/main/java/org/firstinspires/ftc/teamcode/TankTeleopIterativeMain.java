package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwareTank;

@TeleOp(name="Hardware Tank: TankTeleOpMain", group="Iterative Opmode")
public class TankTeleopIterativeMain extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    HardwareTank robot = new HardwareTank();

    private double maxDriveTrain;
    private double maxIntakeSystem;
    private boolean drift = true;
    private double halfSpeed = 1;       //current speed reduction coefficient.  1 at normal power.
    private double flyWheelDelta = .1;
    private double flyWheelPower = robot.defaultFlyPower;

    private double kP = 9000000.0;
    private double kI = 0.0;
    private double kD = 10000.0;

    private double integral = 0.0;
    private double derivative = 0.0;

    private double motorOut = 0.0;
    private double fTarget = 7.5e-7;
    private double fVelocity = 0.0;
    private double fError = 0.0;
    private double fLastError = 0.0;
    private double tbh = 0.0;

    private int fEncoder = 0;
    private int fLastEncoder = 0;

    private long fVelocityTime = 0;
    private long fLastVelocityTime = 0;

    private double tolerance = 0.5e-6;

    private double targetVoltage = 13;
    private double voltage;

    private String particlePref;
    private String beaconPref;
    private String capBallPref;
    private String parkingPref;
    private String alliance;

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
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("left",  robot.leftDrivePower);
        telemetry.addData("right", robot.rightDrivePower);
        telemetry.addData("liveFly",robot.liveFlyPowerSetting);
        telemetry.addData("FlyWheel2", robot.flyWheelMotor2.getPower());
        telemetry.addData("flyWheel1", robot.flyWheelMotor1.getPower());
        telemetry.addData("flyWheelSysPower", robot.systemFlyPower);
        telemetry.addData("spin1Motor", robot.spin1Motor.getPower());
        telemetry.addData("spin2Motor", robot.spin2Motor.getPower());

        getAutonomousPrefs();



        robot.leftDrivePower = 0;
        robot.rightDrivePower = 0;
        robot.innerIntakePower = 0;
        robot.outerIntakePower = 0;
        robot.systemFlyPower = robot.defaultFlyPower;


        robot.flyWheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //Linear Slide Movement Configuration, Currently Sketchy
        //Drive Train Joystick Declaration
        robot.leftDrivePower  = gamepad1.left_stick_y;
        robot.rightDrivePower = gamepad1.right_stick_y;

    }

//Basically adjusts the power based upon the current speed which is based upon encoder ticks over time.
    public void bangBang()
    {
        fVelocityTime = System.nanoTime();
        fEncoder = robot.flyWheelMotor1.getCurrentPosition();
        fVelocity = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);

        if(fVelocity >= (fTarget + tolerance))
        {
            setFPower(robot.minBangValue);
        }

        else if(fVelocity < (fTarget - tolerance))
        {
            setFPower(robot.maxBangValue);
        }

        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;
    }


    private void setFPower(double power)
    {
        robot.flyWheelMotor1.setPower(power);
        robot.flyWheelMotor2.setPower(power);
    }
//prints current velocity of the flywheels.
    private void printVelocity()
    {
        fVelocity = System.nanoTime();
        fEncoder = robot.flyWheelMotor1.getCurrentPosition();
        fVelocity = (double)(fEncoder - fLastEncoder)/ (fVelocity - fLastVelocityTime);

        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;

    }

    private void calculatePID()
    {
        fVelocityTime = System.nanoTime();
        fEncoder = robot.flyWheelMotor1.getCurrentPosition();
        fVelocity = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);
        fError = fTarget - fVelocity;

        integral += fError;
        if(fError == 0)
        {
            integral = 0;
        }

        if(Math.abs(fError) > 50)
        {
            integral = 0;
        }

        derivative = fError - fLastError;

        fLastError = fError;
        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;

        motorOut = (kP * fError) + (kI * integral) + (kD * derivative);

        motorOut = Range.clip(motorOut, 0.0, 1.0);

        telemetry.addData("1", "kP " + (kP * fError));
        telemetry.addData("2", "Error " + fError);
        telemetry.addData("3", "Time " + fVelocityTime);
        telemetry.addData("4", "Encoder " + fEncoder);
        telemetry.addData("5", "Last Encoder " + fLastEncoder);
        telemetry.addData("6", "Encoder Change " + (fEncoder - fLastEncoder));
        telemetry.addData("7", "Time Change " + (fVelocityTime - fLastVelocityTime));
        telemetry.addData("8", "Velocity " + fVelocity);
        telemetry.addData("9", "Result " + motorOut);
        telemetry.update();

        setFPower(motorOut);
    }

    @Override
    public void init_loop() {
    }

//A backup just incase bang bang doesn't work, we use this instead which compensates flywheel speed based upon battery power.
public void voltageProportional()
{
    voltage = robot.getBatteryVoltage();
    double kP = .15;
    double error = targetVoltage - voltage;
    motorOut = (error * kP) + .55;
    motorOut = Range.clip(motorOut, 0, 1);
    setFPower(motorOut);
}
    @Override
    public void start() {
        runtime.reset();
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    /*
    In this particular method, we use a colour sensor that is placed inside our intake system,
    and we use it in order to determine whether or not our robot has intake the incorrect colour in order to spit it out.
    In combination with the colour sensor an autonomous menu on our robot controller is used in order to select what Alliance
     we are on to make sure we are spitting out the correct balls.
     */
    public void colourSensorCheck()
    {
        int[] colourSensor = robot.muxColor.getCRGB(robot.ports[2]);

        if (colourSensor[3] > 6000 && alliance.equals("Red Alliance"))
        {
            double tempTime = runtime.seconds()+2;

            while (runtime.seconds() < tempTime)
            {
                robot.spin1Motor.setPower(-1);
                robot.spin2Motor.setPower(-1);
            }
            robot.spin1Motor.setPower(0);
            robot.spin2Motor.setPower(0);
        }
        else if (colourSensor[1] > 6000 && alliance.equals("Blue Alliance"))
        {
            double tempTime = runtime.seconds() +2;

            while (runtime.seconds() < tempTime)
            {
                robot.spin1Motor.setPower(-1);
                robot.spin2Motor.setPower(-1);
            }
            robot.spin1Motor.setPower(0);
            robot.spin2Motor.setPower(0);
        }

    }

    @Override
    public void loop() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("left",  robot.leftDrivePower);
        telemetry.addData("right", robot.rightDrivePower);
        telemetry.addData("liveFly",robot.liveFlyPowerSetting);
        telemetry.addData("FlyWheel2", robot.flyWheelMotor2.getPower());
        telemetry.addData("flyWheel1", robot.flyWheelMotor1.getPower());
        telemetry.addData("flyWheelSysPower", robot.systemFlyPower);
        telemetry.addData("spin1Motor", robot.spin1Motor.getPower());
        telemetry.addData("spin2Motor", robot.spin2Motor.getPower());
        telemetry.addData("linearSlide1", robot.linearSlideMotor1.getPower());
        telemetry.addData("linearSlide2", robot.linearSlideMotor2.getPower());
        telemetry.addData("4", "Encoder " + fEncoder);
        telemetry.addData("5", "Last Encoder " + fLastEncoder);
        telemetry.addData("6", "Encoder Change " + (fEncoder - fLastEncoder));
        telemetry.addData("7", "Time Change " + (fVelocityTime - fLastVelocityTime));
        telemetry.addData("8", "Velocity " + fVelocity);
       //All Telemetry decleration

        printVelocity();

        colourSensorCheck();


        robot.innerIntakePower = gamepad2.right_stick_y;
        robot.outerIntakePower = gamepad2.left_stick_y;

        robot.leftDrivePower  = gamepad1.left_stick_y;
        robot.rightDrivePower = gamepad1.right_stick_y;


        //Flywheel Controls as well as intake controls to a certain extent
         if (gamepad2.dpad_right) {
            robot.systemFlyPower = robot.defaultFlyPower;
        } else if (gamepad2.right_trigger > 0) {
             voltageProportional();
        }
         else if (robot.innerIntakePower > 0)
         {
             robot.flyWheelMotor1.setPower(-.2);
             robot.flyWheelMotor2.setPower(-.2);
         }
         else {
            robot.flyWheelMotor1.setPower(0);
            robot.flyWheelMotor2.setPower(0);
        }

        if (gamepad1.dpad_up)
        {
            calculatePID();
        }

        // Movement of linear slides a = extend slides, b = retract slides

        if (gamepad2.a)
        {
            robot.linearSlideMovement(1);
        }
        else if (gamepad2.b)
        {
            robot.linearSlideMovement(-1);
        }
        else
        {
            robot.linearSlideMovement(0);
        }

        if (gamepad1.right_trigger > 0)
        {
            robot.flickServoOut();
        }
        else
        {
            robot.flickServoIn();
        }


        //capBall servos for either latching on to the capball, or for releasing the capball capturing mechanism

        if (gamepad1.a)
        {
            robot.capBallServo2.setPosition(1);
        }
        else
        {
            robot.capBallServo2.setPosition(0);
        }

        //capBall servos for either latching on to the capball, or for releasing the capball capturing mechanism

        if (gamepad1.b)
        {
            robot.capBallServo1.setPosition(1);
        }
        else if (gamepad1.x)
        {
            robot.capBallServo1.setPosition(0);
        }


        //Slowing down speed for the Capturing of Beacons
        if (gamepad2.left_bumper) {
            halfSpeed = .5;
        } else if (gamepad2.left_trigger > 0.25) {
            halfSpeed = .25;
        } else {
            halfSpeed = 1;
        }

        //Normalization of Intake System values, since it is driven by joysticks
        maxIntakeSystem = Math.max(Math.abs(robot.innerIntakePower), Math.abs(robot.outerIntakePower));
        if (maxIntakeSystem > 1.0) {
            robot.innerIntakePower /= maxIntakeSystem;
            robot.outerIntakePower /= maxIntakeSystem;
        }



        //Normalization of the Drive Train Values, since it is also driven by joysticks
        maxDriveTrain = Math.max(Math.abs(robot.leftDrivePower), Math.abs(robot.rightDrivePower));

        if (maxDriveTrain > 1.0) {
            robot.leftDrivePower /= maxDriveTrain;
            robot.rightDrivePower /= maxDriveTrain;
        }

        //Toggle Reverse button for Driver 1
        if (gamepad1.right_bumper)
        {
            robot.leftDrivePower  = gamepad1.right_stick_y *-1;
            robot.rightDrivePower = gamepad1.left_stick_y *-1;

        }
        else if (gamepad1.left_bumper)
        {
            robot.leftDrivePower  = gamepad1.left_stick_y;
            robot.rightDrivePower = gamepad1.right_stick_y;
        }

        robot.leftMotor.setPower(robot.leftDrivePower*halfSpeed);
        robot.rightMotor.setPower(robot.rightDrivePower*halfSpeed);

        robot.spin1Motor.setPower(robot.innerIntakePower);
        robot.spin2Motor.setPower(robot.outerIntakePower);

    telemetry.update();


    }

    @Override
    public void stop() {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.spin1Motor.setPower(0);
        robot.spin2Motor.setPower(0);

        robot.flyWheelMotor1.setPower(0);
        robot.flyWheelMotor2.setPower(0);
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
