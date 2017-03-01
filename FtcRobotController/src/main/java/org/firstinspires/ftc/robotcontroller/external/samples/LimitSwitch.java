package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by rommac100 on 2/19/17.
 */

public class LimitSwitch {

    private DigitalChannel limitSwitch;

    public LimitSwitch(HardwareMap hardwareMap, String limitName)
    {
        limitSwitch = hardwareMap.digitalChannel.get(limitName);
        limitSwitch.setMode(DigitalChannelController.Mode.INPUT);
    }

    public boolean returnState()
    {
     return limitSwitch.getState();
    }
}
