package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.*;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.ParallelAction;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;

public class IntakeSubsystem {

    public enum IntakeSpinState {
        IN, OUT, STOP
    }

    public enum IntakePivotState {
        TRANSFER, GROUND
    }

    public DcMotorEx spin;
    private IntakeSpinState spinState;

    private Servo lPivot, rPivot;
    private IntakePivotState pivotState;

    public RunAction spinIn, spinOut, spinStop, pivotTransfer, pivotGround;

    public IntakeSubsystem(HardwareMap hardwareMap, IntakeSpinState spinState, IntakePivotState pivotState) {
        spin = hardwareMap.get(DcMotorEx.class, "intake");
        lPivot = hardwareMap.get(Servo.class, "lPivot");
        rPivot = hardwareMap.get(Servo.class, "rPivot");
        this.spinState = spinState;
        this.pivotState = pivotState;

        spinIn = new RunAction(this::spinIn);
        spinOut = new RunAction(this::spinOut);
        spinStop = new RunAction(this::spinStop);
        pivotTransfer = new RunAction(this::pivotTransfer);
        pivotGround = new RunAction(this::pivotGround);

    }

    // ----------------- Intake Spin -----------------//

    public void setSpinState(IntakeSpinState spinState, boolean changeStateOnly) {
        if (changeStateOnly) {
            this.spinState = spinState;
        } else {
            if (spinState == IntakeSpinState.IN) {
                spinIn();
            } else if (spinState == IntakeSpinState.OUT) {
                spinOut();
            } else if (spinState == IntakeSpinState.STOP) {
                spinStop();
            }
        }
    }

    public void spinIn() {
        spin.setPower(intakeSpinInPwr);
        this.spinState = IntakeSpinState.IN;
    }

    public void spinOut() {
        spin.setPower(intakeSpinOutPwr);
        this.spinState = IntakeSpinState.OUT;
    }

    public void spinStop() {
        spin.setPower(intakeSpinStopPwr);
        this.spinState = IntakeSpinState.STOP;
    }

    // ----------------- Intake Pivot -----------------//

    public void setPivotState(IntakePivotState pivotState) {
        if (pivotState == IntakePivotState.TRANSFER) {
            pivotTransfer();
        } else if (pivotState == IntakePivotState.GROUND) {
            pivotGround();
        }
    }

    public void switchPivotState() {
        if (pivotState == IntakePivotState.TRANSFER) {
            pivotGround();
            pivotState = IntakePivotState.GROUND;
        } else if (pivotState == IntakePivotState.GROUND) {
            pivotTransfer();
            pivotState = IntakePivotState.TRANSFER;
        }
    }

    public void pivotTransfer() {
        lPivot.setPosition(intakePivotTransferPos);
        rPivot.setPosition(1-intakePivotTransferPos);
        this.pivotState = IntakePivotState.TRANSFER;
    }

    public void pivotGround() {
        lPivot.setPosition(intakePivotGroundPos);
        rPivot.setPosition(1-intakePivotGroundPos);
        this.pivotState = IntakePivotState.GROUND;
    }


    public void init() {
        Actions.runBlocking(new ParallelAction(pivotTransfer, spinStop));

    }
    public void start() {
        Actions.runBlocking(new ParallelAction(pivotTransfer, spinStop));
    }
}