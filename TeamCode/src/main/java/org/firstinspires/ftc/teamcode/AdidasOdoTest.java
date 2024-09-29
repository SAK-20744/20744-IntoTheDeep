package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class AdidasOdoTest<OdoPod> extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Object myOtos = hardwareMap.get(OrientationSensor.class,"OdoPod");
        waitForStart();

        while (opModeIsActive()) {
            myOtos.getPosition().x
        }
    }
}