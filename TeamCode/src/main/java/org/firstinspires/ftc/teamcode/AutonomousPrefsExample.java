package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Autonomous Prefs Example", group = "TeleOp")
//@Disabled
public class AutonomousPrefsExample extends LinearOpMode
{
    private String particlePref;
    private String beaconPref;
    private String capBallPref;
    private String parkingPref;
    private String alliance;

    @Override
    public void runOpMode()
    {
        //Wait for the start button to be pressed.
        waitForStart();

        getAutonomousPrefs();
        telemetry.addLine("Particles: " + particlePref);
        telemetry.addLine("Beacons: " + beaconPref);
        telemetry.addLine("Cap Ball: " + capBallPref);
        telemetry.addLine("Parking: " + parkingPref);
        telemetry.addLine("Alliance: " + alliance);
        telemetry.update();

        while (opModeIsActive())
        {
            //Do stuff
        }
    }

    private void getAutonomousPrefs()
    {
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        particlePref = preferences.getString("How Many Particles Should We Shoot?", "");
        beaconPref = preferences.getString("Which beacons should we activate?", "");
        capBallPref = preferences.getString("Should we bump the cap ball off the center vortex?", "");
        parkingPref = preferences.getString("Where should we park?", "");
        alliance = preferences.getString("Which alliance are we on?", "");
    }
}