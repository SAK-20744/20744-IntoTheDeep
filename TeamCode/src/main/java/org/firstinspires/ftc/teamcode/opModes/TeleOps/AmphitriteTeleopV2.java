//package org.firstinspires.ftc.teamcode.opModes.TeleOps;
//
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.CLAW_CLOSED;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.CLAW_OPEN;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.DOOR_CLOSED;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.DOOR_OPEN;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.EXTENDO_EXTENDED;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.EXTENDO_RETRACTED;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.E_RETRACT_POWER;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.INTAKE_IN;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.INTAKE_OFF;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.INTAKE_OUT;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.LDIFFY_CLIPPING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.LDIFFY_SCORING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.LDIFFY_TRANSFERING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.LDIFFY_WALL;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.LIFT_HIGH_BASKET;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.LIFT_HIGH_RUNG;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.LIFT_MID_BASKET;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.LIFT_RETRACTED;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.RAIL_CLIPPING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.RAIL_SCORING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.RAIL_TRANSFERING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.RAIL_WALL;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.RDIFFY_CLIPPING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.RDIFFY_SCORING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.RDIFFY_TRANSFERING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.RDIFFY_WALL;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.ROLL_DEPO;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.ROLL_TRANSFERING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.WRIST_CLOSE_INTAKING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.WRIST_INTAKING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.WRIST_TRANSFERING;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.WRIST_UP;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.clipRange;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.ed;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.ei;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.ep;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.ld;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.li;
//import static org.firstinspires.ftc.teamcode.config.RobotConstants.lp;
//import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
//import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
//import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
//import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.rightRearMotorName;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.subsystems.Deposit.ClawSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Deposit.DiffySubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Deposit.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Deposit.RailSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Deposit.RollSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Intake.ExtendSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.util.Timer;
//
//@Config
//@TeleOp(name = "gpt", group = "Competition")
//public class AmphitriteTeleopV2 extends OpMode {
//
//    public LiftSubsystem lift;
//    public ExtendSubsystem extend;
//
//    private Follower follower;
//    private DcMotorEx leftFront;
//    private DcMotorEx leftRear;
//    private DcMotorEx rightFront;
//    private DcMotorEx rightRear;
//
//    private boolean specimenmode = false;
//
//    private double looptime = 0;
//    private Servo wrist, door, roll, claw, rail, lDiffy, rDiffy;
//    private DcMotorEx lLift, rLift, intake, extendo;
//    private DigitalChannel liftLimit, extendoLimit;
//
//    public Timer scoringTimer = new Timer();
//
////    RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "Color");
//
////    public static double
////            INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0.3,
////            LDIFFY_TRANSFERING = 0.18, LDIFFY_CLIPPING = 0.4, LDIFFY_SCORING = 0.65, LDIFFY_WALL = 1,
////            RDIFFY_TRANSFERING = 0.18, RDIFFY_CLIPPING = 0, RDIFFY_SCORING = 0.5, RDIFFY_WALL = 0.73,
////            CLAW_CLOSED = 0.55, CLAW_OPEN = 0.25,
////            WRIST_TRANSFERING = 0.82, WRIST_UP = 0.4, WRIST_INTAKING = 0.13,
////            DOOR_OPEN = 0.6, DOOR_CLOSED = 0.2,
////            ROLL_DEPO = 0.55, ROLL_TRANSFERING = 0,
////            RAIL_TRANSFERING = 0, RAIL_WALL= 0.075, RAIL_SCORING = 0, RAIL_CLIPPING = 0.85,
////            E_RETRACT_POWER = -0;
////
////    public static int
////            LIFT_RETRACTED = -25, LIFT_MID_BASKET = 500, LIFT_HIGH_BASKET = 1250, LIFT_HIGH_RUNG = 443, LIFT_MID_RUNG = 300, clipRange = 250,
////            EXTENDO_RETRACTED = -10, EXTENDO_EXTENDED = 450;
//
//    private int liftTarget = LIFT_RETRACTED;
//    private int liftLiftedTarget = LIFT_HIGH_BASKET;
//    private int extendoTarget = EXTENDO_RETRACTED;
//
//    private double lDiffyTarget = LDIFFY_TRANSFERING;
//    private double rDiffyTarget = RDIFFY_TRANSFERING;
//
//    private double clawTarget = CLAW_OPEN;
//    private double wristTarget = WRIST_TRANSFERING;
//    private double doorTarget = DOOR_OPEN;
//    private double intakePower = INTAKE_OFF;
//    private double rollTarget = ROLL_TRANSFERING;
//
//    private double railTarget = RAIL_TRANSFERING;
//
//    private PIDController liftPID;
////    public static double lp = -0.007, li = 0, ld = 0.000003;
//
//    private PIDController extendoPID;
////    public static double ep = 0.038, ei = 0, ed = 0.000005;
//
//    @Override
//    public void init() {
//
//        follower = new Follower(hardwareMap);
//
//        lift = new LiftSubsystem(hardwareMap, telemetry, true);
//        extend = new ExtendSubsystem(hardwareMap, telemetry);
//
//        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
//        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
//        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
//        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
//
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        liftPID = new PIDController(lp, li, ld);
//        extendoPID = new PIDController(ep, ei, ed);
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
//        extendoLimit = hardwareMap.get(DigitalChannel.class, "extendoLimit");
//
//        lLift = hardwareMap.get(DcMotorEx.class, "lLift");
//        rLift = hardwareMap.get(DcMotorEx.class, "rLift");
//        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
//
//        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        wrist = hardwareMap.get(Servo.class, "wrist");
//        roll = hardwareMap.get(Servo.class, "roll");
//        door = hardwareMap.get(Servo.class, "door");
//        claw = hardwareMap.get(Servo.class, "claw");
//        lDiffy = hardwareMap.get(Servo.class, "lDiffy");
//        rDiffy = hardwareMap.get(Servo.class, "rDiffy");
//        rail = hardwareMap.get(Servo.class, "rail");
//
//        lLift.setDirection(DcMotorSimple.Direction.REVERSE);
//        extendo.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        intakePower = INTAKE_OFF;
//
//        door.setPosition(doorTarget);
//        roll.setPosition(rollTarget);
//        rail.setPosition(railTarget);
//        wrist.setPosition(wristTarget);
////        extendo.setTargetPosition(extendoTarget);
////        lLift.setTargetPosition(liftTarget);
////        rLift.setTargetPosition(liftTarget);
//        claw.setPosition(clawTarget);
//        lDiffy.setPosition(lDiffyTarget);
//        rDiffy.setPosition(rDiffyTarget);
//        rail.setPosition(railTarget);
//        intake.setPower(intakePower);
//
//    }
//
//    public void init_loop(){
//
//        if (liftLimit.getState()){
//            lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//
//        if (!extendoLimit.getState()){
//            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//
//        intake.setPower(0);
//
//        telemetry.addData("lLift Current", lLift.getCurrentPosition());
//        telemetry.addData("rLift Current", rLift.getCurrentPosition());
//        telemetry.addData("Extendo Current", extendo.getCurrentPosition());
//        telemetry.addData("Lift Limit", liftLimit.getState());
//        telemetry.addData("Extendo Limit", extendoLimit.getState());
//        telemetry.update();
//    }
//
//    @Override
//    public void loop() {
//
//        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = gamepad1.right_stick_x * 0.7;
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//
//        leftFront.setPower(frontLeftPower);
//        leftRear.setPower(backLeftPower);
//        rightFront.setPower(frontRightPower);
//        rightRear.setPower(backRightPower);
//
//        if (gamepad1.left_bumper && !brushlands.detectSample()) {
//            extendoTarget = EXTENDO_EXTENDED;
//
//            if (gamepad1.right_bumper) {
//                intakePower = INTAKE_IN;
//                wristTarget = WRIST_INTAKING;
//            } else {
//                intakePower = INTAKE_OFF;
//                wristTarget = WRIST_UP;
//            }
//        } else {
//            extendoTarget = EXTENDO_RETRACTED;
//
//            if (gamepad1.right_bumper) {
//                intakePower = INTAKE_IN;
//                wristTarget = WRIST_CLOSE_INTAKING;
//            } else {
//                intakePower = INTAKE_OFF;
//                wristTarget = WRIST_TRANSFERING;
//            }
//        }
//
//        if (gamepad2.dpad_down || gamepad1.dpad_down) {
//            liftLiftedTarget = LIFT_MID_BASKET;
//        }
//        if (gamepad2.dpad_up || gamepad1.dpad_up) {
//            liftLiftedTarget = LIFT_HIGH_BASKET;
//        }
//
//        if (gamepad1.a) {
//            clawTarget = CLAW_OPEN;
//            if (claw.isAt(CLAW_OPEN)) {
//                railTarget = RAIL_CLIPPING;
//                lDiffyTarget = LDIFFY_TRANSFERING;
//                rDiffyTarget = RDIFFY_TRANSFERING;
//                rollTarget = ROLL_TRANSFERING;
//                if (rail.isAt(RAIL_CLIPPING)) {
//                    liftTarget = LIFT_RETRACTED;
//                    if (lift.isAtTarget()) {
//                        scoringTimer.resetTimer();
//                    }
//                    if (scoringTimer.getElapsedTimeSeconds() > 0.5) {
//                        railTarget = RAIL_TRANSFERING;
//                    }
//                }
//            }
//        }
//
//        if (gamepad1.b) {
//            liftTarget = liftLiftedTarget;
//            railTarget = RAIL_CLIPPING;
//            if (rail.isAt(RAIL_CLIPPING)) {
//                lDiffyTarget = LDIFFY_SCORING;
//                rDiffyTarget = RDIFFY_SCORING;
//                rollTarget = ROLL_DEPO;
//                if (lift.isAtTarget()) {
//                    railTarget = RAIL_SCORING;
//                }
//            }
//        }
//
//// Automated Transferring Logic
//        if (lift.isAtTarget() && extend.isAtTarget() && wrist.isAt(WRIST_TRANSFERING) && brushlands.detectSample()) {
//            clawTarget = CLAW_CLOSED;
//        }
//
//        door.setPosition(doorTarget);
//        roll.setPosition(rollTarget);
//        rail.setPosition(railTarget);
//        wrist.setPosition(wristTarget);
//        extendo.setTargetPosition(extendoTarget);
//        lLift.setTargetPosition(liftTarget);
//        rLift.setTargetPosition(liftTarget);
//        claw.setPosition(clawTarget);
//        lDiffy.setPosition(lDiffyTarget);
//        rDiffy.setPosition(rDiffyTarget);
//        rail.setPosition(railTarget);
//        intake.setPower(intakePower);
//
//        liftPID.setPID(lp,li,ld);
//        int pos = rLift.getCurrentPosition();
//        double power = liftPID.calculate(pos, liftTarget);
//
//        if (liftLimit.getState()){
//            lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//
//        lLift.setPower(power);
//        rLift.setPower(power);
//
//        extendoPID.setPID(ep,ei,ed);
//        int expos = extendo.getCurrentPosition();
//        double expower = extendoPID.calculate(expos, extendoTarget);
//        if (!extendoLimit.getState()){
//            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            if(expower < 0)
//                expower = E_RETRACT_POWER;
//        }
//        telemetry.addData("exPower", expower);
//        extendo.setPower(expower);
//
//
////        NormalizedRGBA colors = sensor.getNormalizedColors();
////        telemetry.addData("rgb: ", colors.red + " " + colors.blue + " " + colors.green);
//
//        telemetry.addData("Claw Pos", claw.getPosition());
//        telemetry.addData("Wrist Pos", wrist.getPosition());
//        telemetry.addData("lLift Current", lLift.getCurrentPosition());
//        telemetry.addData("rLift Current", rLift.getCurrentPosition());
//        telemetry.addData("Extendo Current", extendo.getCurrentPosition());
//        telemetry.addData("Lift Limit", liftLimit.getState());
//        telemetry.addData("Extendo Limit", extendoLimit.getState());
//        telemetry.addData("Intake", intake.getPower());
//        telemetry.addData("Specimen?", specimenmode);
//
//        double loop = System.nanoTime();
//        telemetry.addData("hz ", 1000000000 / (loop - looptime));
//        looptime = loop;
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        super.start();
//    }
//
//    @Override
//    public void stop() { super.stop(); }
//
//}
