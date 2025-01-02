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

    public enum DoorState {
        CLOSED, OPEN
    }

    private DcMotorEx spin;
    private IntakeSpinState spinState;
    private Servo lPivot, door;
    private IntakePivotState pivotState;
    private DoorState doorState;

    public RunAction spinIn, spinOut, spinStop, pivotTransfer, pivotGround, openDoor, closeDoor;

    public IntakeSubsystem(HardwareMap hardwareMap, IntakeSpinState spinState, IntakePivotState pivotState, DoorState doorState) {
        spin = hardwareMap.get(DcMotorEx.class, "intake");
        lPivot = hardwareMap.get(Servo.class, "wrist");
        door = hardwareMap.get(Servo.class, "door");
        this.spinState = spinState;
        this.pivotState = pivotState;
        this.doorState = doorState;

        spinIn = new RunAction(this::spinIn);
        spinOut = new RunAction(this::spinOut);
        spinStop = new RunAction(this::spinStop);
        openDoor = new RunAction(this::doorOpen);
        closeDoor = new RunAction(this::doorClosed);
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
        this.pivotState = IntakePivotState.TRANSFER;
    }

    public void pivotGround() {
        lPivot.setPosition(intakePivotGroundPos);
        this.pivotState = IntakePivotState.GROUND;
    }

    public void doorOpen() {
        door.setPosition(DOOR_OPEN);
        this.doorState = DoorState.OPEN;
    }

    public void doorClosed() {
        door.setPosition(DOOR_CLOSED);
        this.doorState = DoorState.CLOSED;
    }

    public void init() {
        Actions.runBlocking(new ParallelAction(pivotTransfer, spinStop));

    }
    public void start() {
        Actions.runBlocking(new ParallelAction(pivotTransfer, spinStop));
    }
}