package org.firstinspires.ftc.teamcode.subsystems.Intake;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.DOOR_CLOSED;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.DOOR_OPEN;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.WRIST_INTAKING;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.WRIST_TRANSFERING;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.*;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opModes.TeleOps.Red;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.ParallelAction;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;

public class NewIntakeSubsystem {

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
    private RevColorSensorV3 colorSense;
    private Servo wrist, door;
    private IntakePivotState pivotState;
    private COLOR colorDetected;
    private DoorState doorState;

    public RunAction spinIn, spinOut, spinStop, pivotTransfer, pivotGround, openDoor, closeDoor;

    public NewIntakeSubsystem(HardwareMap hardwareMap, IntakeSpinState spinState, IntakePivotState pivotState, COLOR colordetected) {
        spin = hardwareMap.get(DcMotorEx.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        door = hardwareMap.get(Servo.class, "door");
        this.spinState = spinState;
        this.pivotState = pivotState;
//        this.doorState = doorState;

        this.colorDetected = colordetected;

        spinIn = new RunAction(this::spinIn);
        spinOut = new RunAction(this::spinOut);
        spinStop = new RunAction(this::spinStop);
        openDoor = new RunAction(this::doorOpen);
        closeDoor = new RunAction(this::doorClosed);
        pivotTransfer = new RunAction(this::pivotTransfer);
        pivotGround = new RunAction(this::pivotGround);

    }

    public enum COLOR {
        RED,
        BLUE,
        YELLOW,
        NONE
    }

    private void updateColor() {
        NormalizedRGBA colors = colorSense.getNormalizedColors();

        if (colors.green < greenVal && colors.red < redVal && colors.blue < blueVal)
            colorDetected = COLOR.NONE;
        else if (colors.green > colors.red && colors.green > colors.blue)
            colorDetected = COLOR.YELLOW;
        else if (colors.red > colors.blue && colors.red > colors.green)
            colorDetected = COLOR.RED;
        else
            colorDetected = COLOR.BLUE;
    }

    public COLOR getColorDetected() {

        NormalizedRGBA colors = colorSense.getNormalizedColors();

        if (colors.green < greenVal && colors.red < redVal && colors.blue < blueVal)
            colorDetected = COLOR.NONE;
        else if (colors.green > colors.red && colors.green > colors.blue)
            colorDetected = COLOR.YELLOW;
        else if (colors.red > colors.blue && colors.red > colors.green)
            colorDetected = COLOR.RED;
        else
            colorDetected = COLOR.BLUE;

        return colorDetected;
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
        spin.setPower(INTAKE_IN);
        this.spinState = IntakeSpinState.IN;
    }

    public void spinOut() {
        spin.setPower(INTAKE_OUT);
        this.spinState = IntakeSpinState.OUT;
    }

    public void spinStop() {
        spin.setPower(INTAKE_OFF);
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
        wrist.setPosition(WRIST_TRANSFERING);
        this.pivotState = IntakePivotState.TRANSFER;
    }

    public void pivotGround() {
        wrist.setPosition(WRIST_INTAKING);
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