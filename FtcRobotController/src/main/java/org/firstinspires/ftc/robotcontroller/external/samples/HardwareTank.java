package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcontroller.external.samples.MultiplexColorSensor;
import org.firstinspires.ftc.robotcontroller.external.samples.PingDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * 
 *
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareTank
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  spin1Motor = null;
    public DcMotor  spin2Motor = null;
    public DcMotor  flyWheelMotor1 = null;
    public DcMotor  flyWheelMotor2 = null;
    public DcMotor linearSlideMotor1 = null;
    public DcMotor linearSlideMotor2 = null;
    public Servo    capBallServo1 = null;
    public Servo    capBallServo2 = null;
    public Servo    flickerServo1 = null;
    public Servo    flickerServo2 = null;

    public DeviceInterfaceModule device = null;
    public ColorSensor colourSensor = null;
    public PingDistanceSensor ping = null;
    public LimitSwitch limit1 = null;
    public LimitSwitch limit2 = null;

    public VoltageSensor voltageSensor;

    public final double ticksPerInch = 53.4776;
    public double leftDrivePower;       //power level for left side drive train motor
    public double rightDrivePower;      //power level for right side drive train motor
    public double innerIntakePower;     //power level for the inner intake
    public double outerIntakePower;     //power level for the outer intake
    public double systemFlyPower;       //current power level for fly motors
    public double defaultFlyPower = .7;
    public double liveFlyPowerSetting = defaultFlyPower;

    public double voltage = 0;

    public double minBangValue = .4;// for bangbang for the flywheels
    public double maxBangValue = .5; // for bangbang for the flywheels

    public MultiplexColorSensor muxColor;
    public int[] ports = {0, 1};

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTank() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left_drive");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor  = hwMap.dcMotor.get("right_drive");
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spin1Motor = hwMap.dcMotor.get("spin1");
        spin2Motor = hwMap.dcMotor.get("spin2");
        flyWheelMotor1 = hwMap.dcMotor.get("fly1");
        flyWheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelMotor2 = hwMap.dcMotor.get("fly2");
        flyWheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        spin1Motor.setDirection(DcMotor.Direction.REVERSE);
        spin2Motor.setDirection(DcMotor.Direction.REVERSE);
        flyWheelMotor1.setDirection(DcMotor.Direction.REVERSE);
        device = hwMap.deviceInterfaceModule.get("deviceINT");
        colourSensor = hwMap.colorSensor.get("colour_sensor");

        ping = new PingDistanceSensor(hwMap,"ping");
        limit1 = new LimitSwitch(hwMap, "limit1");
        limit2 = new LimitSwitch(hwMap, "limit2");


        capBallServo1 = hwMap.servo.get("cap1");
        capBallServo2 = hwMap.servo.get("cap2");

        flickerServo1 = hwMap.servo.get("flick1");
        flickerServo2 = hwMap.servo.get("flick2");

        linearSlideMotor1 = hwMap.dcMotor.get("linear1");
        linearSlideMotor2 = hwMap.dcMotor.get("linear2");
        linearSlideMotor1.setDirection(DcMotor.Direction.FORWARD);
        linearSlideMotor2.setDirection(DcMotor.Direction.REVERSE);

        int milliSeconds = 48;
        muxColor = new MultiplexColorSensor(hwMap, "mux", "ada",
                ports, milliSeconds,
                MultiplexColorSensor.GAIN_16X);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        spin1Motor.setPower(0);
        spin2Motor.setPower(0);
        flyWheelMotor1.setPower(0);
        flyWheelMotor2.setPower(0);
        linearSlideMotor1.setPower(0);
        linearSlideMotor2.setPower(0);


        capBallServo1.setPosition(0);
        capBallServo2.setPosition(0);

flickServoIn();



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.


        // Define and initialize ALL installed servos.

    }
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    public void flickServoOut()
    {
        flickerServo1.setPosition(.75);
        flickerServo2.setPosition(.2);
    }
    public void flickServoIn()
    {
        flickerServo1.setPosition(.15);
        flickerServo2.setPosition(.88);
    }

    public void driveMotors(double powerLeft, double powerRight)
    {
        leftMotor.setPower(powerLeft);
        rightMotor.setPower(powerRight);
    }

    public void linearSlideMovement(double power)
    {
        linearSlideMotor1.setPower(power);
        linearSlideMotor2.setPower(power);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */

    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
