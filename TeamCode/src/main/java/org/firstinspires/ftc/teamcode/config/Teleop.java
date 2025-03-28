//package org.firstinspires.ftc.teamcode.config;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.subsystems.Deposit.ClawSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Deposit.DiffySubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Deposit.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Deposit.RailSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Deposit.RollSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Intake.ExtendSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
//
//public class Teleop {
//
//    public ClawSubsystem claw;
//    public ClawSubsystem.ClawState clawState;
//    public LiftSubsystem lift;
//    public ExtendSubsystem extend;
//    public IntakeSubsystem intake;
//    public IntakeSubsystem.IntakeSpinState intakeSpinState;
//    public IntakeSubsystem.IntakePivotState intakePivotState;
//    public RailSubsystem rail;
//    public RailSubsystem.railState railState;
//    public RollSubsystem roll;
//    public RollSubsystem.RollState rollState;
//    public DiffySubsystem diffy;
//    public DiffySubsystem.diffyState diffyState;
//
//    private boolean transferring = false;
//    private boolean scoring = false;
//
//    public Teleop(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, boolean isBlue, boolean isBucket) {
//        claw = new ClawSubsystem(hardwareMap, clawState);
//        lift = new LiftSubsystem(hardwareMap, telemetry, false);
//        extend = new ExtendSubsystem(hardwareMap, telemetry);
//        intake = new IntakeSubsystem(hardwareMap, intakeSpinState, intakePivotState);
//        diffy = new DiffySubsystem(hardwareMap, diffyState);
//        rail = new RailSubsystem(hardwareMap, railState);
//        roll = new RollSubsystem(hardwareMap, rollState);
//    }
//
//    public void update(boolean aButtonPressed) {
//        if (aButtonPressed) {
//            initiateTransfer();
//        }
//        checkTransferCompletion();
//        checkScoringCompletion();
//    }
//
//    private void initiateTransfer() {
//        claw.open();
//        transferring = true;
//    }
//
//    private void checkTransferCompletion() {
//        if (transferring && claw.isOpenConfirmed()) {
//            railDiffy.moveToTransferringPosition();
//            lift.lower();
//
//            if (lift.isFullyLowered() && extendo.isFullyRetracted() && wrist.isFullyUp() && intake.hasSample()) {
//                claw.close();
//                transferring = false;
//                scoring = true;
//            }
//        }
//    }
//
//    private void checkScoringCompletion() {
//        if (scoring && claw.isHoldingObject()) {
//            lift.raiseToScoringHeight();
//            if (lift.isAtScoringHeight()) {
//                railDiffy.moveToScoringPosition();
//                scoring = false;
//            }
//        }
//    }
//}
