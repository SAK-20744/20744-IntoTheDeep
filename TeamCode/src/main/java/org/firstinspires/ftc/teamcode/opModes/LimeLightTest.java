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
        LL = hardwareMap.get(Limelight3A.class, "ll3");
        LL.pipelineSwitch(0);
        LL.start();
    }

    @Override
    public void loop() {
        LL.pipelineSwitch(0);
        LLResult result = LL.getLatestResult();
        LL.pipelineSwitch(3);
        LLResult aResult = LL.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D YellowSamplePose = result.getBotpose();
                double detectedX = result.getTx();
                double detectedY = result.getTy();
                telemetry.addData("tx", detectedX);
                telemetry.addData("ty", detectedY);
            }
        }
        if (aResult != null) {
            if (result.isValid()) {
                telemetry.addData("ATPos", result.getBotpose_MT2());
            }
        }
        telemetry.update();
    }
}