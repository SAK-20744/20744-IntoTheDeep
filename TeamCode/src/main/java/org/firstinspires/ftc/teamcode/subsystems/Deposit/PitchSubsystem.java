package org.firstinspires.ftc.teamcode.subsystems.Deposit;

import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.armInPos;
import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.armOutPos;
import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.pitchInPos;
import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.pitchOutPos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;

public class PitchSubsystem {

    public enum PitchState {
        IN, OUT
    }

    private Servo pitch;
    private PitchState state;
    public RunAction pitchIn, pitchOut;

    public PitchSubsystem(HardwareMap hardwareMap, PitchState pitchState) {
        pitch = hardwareMap.get(Servo.class, "pitch");
        this.state = pitchState;

        pitchIn = new RunAction(this::setPitchIn);
        pitchOut = new RunAction(this::setPitchOut);
    }

    public void setPos(double armPos) {
        pitch.setPosition(armPos);
    }

    public void setState(PitchState pitchState) {
        if (pitchState == PitchState.IN) {
            pitch.setPosition(pitchInPos);
            this.state = PitchState.IN;
        } else if (pitchState == PitchState.OUT) {
            pitch.setPosition(pitchOutPos);
            this.state = PitchState.OUT;
        }
    }

    public void setPitchIn() {
        setState(PitchState.IN);
    }

    public void setPitchOut() {
        setState(PitchState.OUT);
    }

    public void init() {
        Actions.runBlocking(pitchIn);
    }

    public void start() {
        Actions.runBlocking(pitchIn);
    }



}