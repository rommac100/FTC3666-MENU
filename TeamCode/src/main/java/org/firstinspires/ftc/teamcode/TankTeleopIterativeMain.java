package org.firstinspires.ftc.teamcode;

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

    double tbhI = 300.0;
    private double kP = 9000000.0;
    private double kI = 0.0;
    //private double kD = 380000;
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

    private double place = 0.1;

    private boolean firstCross;

    private double tolerance = 0.5e-6;

    private double targetVoltage = 13;
    private double voltage;

    public boolean toggleFlickers = false;

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



        robot.leftDrivePower = 0;
        robot.rightDrivePower = 0;
        robot.innerIntakePower = 0;
        robot.outerIntakePower = 0;
        robot.systemFlyPower = robot.defaultFlyPower;


        //Having flywheels using PID instead just power.
        //Having flywheels using PID instead just power.
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

    public void adjustPID()
    {
        if(gamepad2.dpad_up)
        {
            kP += (1.0 * place);
        }

        if(gamepad2.dpad_down)
        {
            kP -= (1.0 * place);
        }

        if(gamepad1.dpad_left)
        {
            place *= 10.0;
        }

        if(gamepad2.left_bumper)
        {
            place *= 0.1;
        }
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

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flyWheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flyWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    public double colourSensorCheck(String teamColour)
    {
        int red = robot.colourSensor.red();
        int blue = robot.colourSensor.blue();

        int x = 0;


        //averaging loop
        telemetry.addData("redVal", red);
        telemetry.addData("blueVal", blue);

        telemetry.update();

        //Different conditionals relating the colour sensor output + team Alliance colour.
        if (teamColour.equals("blue")) {
            if ((red) > blue) {
                return .2;
            }
            else if (red < blue)
            {
                return .9;
            }
            else
            {
                return colourSensorCheck("blue");
            }
        }
        else if (teamColour.equals("red"))
        {
            if ((red) > blue)
            {
                return .2;
            }
            else if (red < blue)
            {
                return .9;
            }
            else {
                return colourSensorCheck("blue");
            }
        }
        return .5;
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



         //adjustPID();
        robot.innerIntakePower = gamepad2.right_stick_y;
        robot.outerIntakePower = gamepad2.left_stick_y;

        robot.leftDrivePower  = gamepad1.left_stick_y;
        robot.rightDrivePower = gamepad1.right_stick_y;


        //Flywheel Conditionals, allows the variability of power/speed
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

        // Movement of linear slides - self explanatory.

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

        //Controlling of Drift - DcMotor Braking configuration
        //Marvin Servo Control, using 180 degree Servo on the Intake side of the Robot

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
