package org.firstinspires.ftc.teamcode.subsystems.Deposit;

import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.armInPos;
import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.armOutPos;
import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.clawClose;
import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.clawOpen;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;

public class DepoArmSubsystem {

    public enum DepoState {
        IN, OUT
    }

    private Servo depoArm;
    private DepoState state;
    public RunAction armIn, armOut;

    public DepoArmSubsystem(HardwareMap hardwareMap, DepoState depoState) {
        depoArm = hardwareMap.get(Servo.class, "leftV4B");
        this.state = depoState;

        armIn = new RunAction(this::setArmIn);
        armOut = new RunAction(this::setArmOut);
    }

    public void setPos(double armPos) {
        depoArm.setPosition(armPos);
    }

    public void setState(DepoState clawState) {
        if (clawState == DepoState.IN) {
            depoArm.setPosition(armInPos);
            this.state = DepoState.IN;
        } else if (clawState == DepoState.OUT) {
            depoArm.setPosition(armOutPos);
            this.state = DepoState.OUT;
        }
    }

    public void switchState() {
        if (state == DepoState.OUT) {
            setState(DepoState.IN);
        } else if (state == DepoState.IN) {
            setState(DepoState.OUT);
        }
    }

    public void setArmIn() {
        setState(DepoState.IN);
    }

    public void setArmOut() {
        setState(DepoState.OUT);
    }

    public void init() {
        Actions.runBlocking(armIn);
    }

    public void start() {
        Actions.runBlocking(armIn);
    }



}