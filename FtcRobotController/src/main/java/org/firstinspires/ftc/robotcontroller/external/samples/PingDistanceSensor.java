package org.firstinspires.ftc.robotcontroller.external.samples;

/**
 * Created by rommac100 on 2/15/17.
 */
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PingDistanceSensor {

    private DigitalChannel ping;



    public PingDistanceSensor(HardwareMap hardwareMap, String pingName)
    {

        ping = hardwareMap.digitalChannel.get(pingName);
        ping.setMode(DigitalChannelController.Mode.OUTPUT);
    }

    public void turnOn()
    {

    }




    public double getDistanceInches(ElapsedTime runtime)
    {
        boolean tick = true;
        double cutOffTime = runtime.milliseconds()+2/1000;

        double timing = 0;

        ping.setMode(DigitalChannelController.Mode.OUTPUT);
        ping.setState(false);
        while(runtime.milliseconds()<cutOffTime); //wait 2ms for the response of the ping.
        cutOffTime = runtime.milliseconds()+3/1000;
        while(runtime.milliseconds()<cutOffTime);
        ping.setState(true);
        cutOffTime = runtime.milliseconds()+30;
        ping.setMode(DigitalChannelController.Mode.INPUT);
        while(runtime.milliseconds()<cutOffTime&&tick)
        {
            if(ping.getState())
            {
                timing =runtime.milliseconds();
                tick=false;
            }
        }
        double inches = timing/1000 / 74 / 2;
        return inches;
    }

    public double getDistanceCM(ElapsedTime runtime) {
        long timeOff = System.nanoTime()+3000;
        long timeOn = System.nanoTime()+15000000;
        ping.setMode(DigitalChannelController.Mode.OUTPUT);

        while (System.nanoTime() < timeOn)
        {
            if (System.nanoTime() <timeOff)
            {
                ping.setState(false);
            }
            else if (System.nanoTime() <timeOn) {
                ping.setState(true);
            }
        }
        ping.setState(false);
        long duration = 0;
        ping.setMode(DigitalChannelController.Mode.INPUT);
        long timeIn = System.nanoTime() + 10000000;
        long timeEnter = 0;
        long timeEnd = 0;
        while (ping.getState()) {
            if (System.nanoTime() < timeIn) {
                break;
            }
        }
        timeIn = System.nanoTime() + 1000000000;

        while (!ping.getState()) {
            timeEnter = System.nanoTime();
            if (System.nanoTime() < timeIn) {
                break;
            }
        }

        timeIn = System.nanoTime() + 1000000000;

        while (ping.getState()) {
            timeEnd = System.nanoTime();
            if (System.nanoTime() < timeIn) {
                break;
            }
        }
        double distance =0;
        if (timeEnd <= 0) {
            distance = 100;
        } else
        {
            duration = timeEnd-timeEnter;
            distance = duration/2/2940000;
        }





        /*
        while (!ping.getState()&& System.nanoTime() < timeIn)
        {

        }
        if (ping.getState()) {
            timeIn = System.nanoTime();
            while (ping.getState() && System.nanoTime() < timeIn+10000000)
            {

            }
            long timeOut = System.nanoTime();

            duration = timeOut-timeIn;
        }

        ping.setMode(DigitalChannelController.Mode.OUTPUT);

*/

        return distance/100;
    }
}
