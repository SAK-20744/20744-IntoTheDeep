package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class LimeLightTest extends OpMode {
    public Limelight3A LL;

    @Override
    public void init() {
        LL = hardwareMap.get(Limelight3A.class, "Limelight");
        LL.pipelineSwitch(0);
        LL.start();
    }

    @Override
    public void loop() {
        LLResult result = LL.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D YellowSamplePose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Sample:", YellowSamplePose.toString());
                telemetry.addData("Image", result);
                telemetry.update();
            }
        }
    }
}