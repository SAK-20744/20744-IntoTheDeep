package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class AdidasOdoTest<OdoPod> extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOS myotos = hardwareMap.get(SparkFunOTOS.class,"sensor_otos");
        SparkFunOTOS.Pose2D Position;
        waitForStart();

        while (opModeIsActive()) {
            Position = myotos.getPosition();
            telemetry.addLine(Position.toString());
            telemetry.update();
        }
    }
}