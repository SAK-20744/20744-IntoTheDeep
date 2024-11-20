package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public abstract class LimeLightTest extends OpMode {
    public Limelight3A LL = hardwareMap.get(Limelight3A.class, "Limelight");

    public void runOpMode() throws InterruptedException {
        LL.pipelineSwitch(0);
        LL.start();
        while (linearOpMode.opModeIsActive()) {
            LLResult result = LL.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    List SamplePosition = result.getColorResults();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Sample:", SamplePosition.toString());
                    telemetry.update();
                }
            }
        }
    }
}