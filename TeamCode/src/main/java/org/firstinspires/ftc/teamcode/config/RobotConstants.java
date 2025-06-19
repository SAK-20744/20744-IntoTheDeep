package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {


    public static double
            INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0.67,
            LDIFFY_TRANSFERING = 0.85, LDIFFY_CLIPPING = 0.56, LDIFFY_SCORING = 0.4375, LDIFFY_WALL = 0.07, LDIFFY_AUTO = 0.4, LDIFFY_NEW_AUTO = 0.6,
            RDIFFY_TRANSFERING = 0.9, RDIFFY_CLIPPING = 1, RDIFFY_SCORING = 0.4, RDIFFY_WALL = 0.02, RDIFFY_AUTO = 0.35, RDIFFY_NEW_AUTO = 0.6,
            CLAW_CLOSED = 0.54, CLAW_OPEN = 0.1, CLAW_SPEC = 0,
            WRIST_TRANSFERING = 1, WRIST_UP = 0.72, WRIST_INTAKING = 0.32, WRIST_CLOSE_INTAKING = 0.28,
            DOOR_OPEN = 0.6, DOOR_CLOSED = 0.2,
            ROLL_DEPO = 0.75, ROLL_TRANSFERING = 0.18, YAW = 0.125,
            LRAIL_TRANSFERING = 0.92, LRAIL_WALL= 0.9, LRAIL_SCORING = 1, LRAIL_CLIPPING = 0,
            RRAIL_TRANSFERING = 0.92, RRAIL_WALL= 0.9, RRAIL_SCORING = 1, RRAIL_CLIPPING = 0,
            E_RETRACT_POWER = -0.5,
            redVal = 0.025, blueVal = 0.02, greenVal = 0.5,
            grabtime = 0.18,
            wristNeeded = 135;

    public static int
            LIFT_RETRACTED = -7, LIFT_MID_BASKET = 500, LIFT_HIGH_BASKET = 1350, LIFT_HIGH_RUNG = 410, LIFT_HIGH_RUNG_ADJUSTED = 458,  LIFT_AUTO_RUNG = 378, LIFT_MID_RUNG = 300, clipRange = 20,
            EXTENDO_RETRACTED = -25, EXTENDO_RETRACTED_TELE = -75, EXTENDO_EXTENDED = 500, EXTENDO_AUTO = 220, EXTENDO_OUTTAKE = 275,
            RANGEFINDERRANGE = 65;

    public static double lp = -0.0085, li = 0.000275, ld = 0;
    public static double autoP = -0.01, autoI = 0.007, autoD = 0.0000;
    public static double ep = 0.02, ei = 0.00007, ed = 0.000005;

}





//public static double clawClose = 0.375;
//    public static double clawOpen = 0;
//    public static double doorClose = 0.93;
//    public static double doorOpen = 0.5;
//    public static double armInPos = 0.365;
//    public static double armOutPos = 0.6;
//    public static double pitchInPos = 0.865;
//    public static double pitchOutPos = 0.5;
//    public static double intakeSpinInPwr = 1;
//    public static double intakeSpinOutPwr = -0.25;
//    public static double intakeSpinStopPwr = 0;
//    public static double intakePivotTransferPos= 0;
//    public static double intakePivotGroundPos = 0.882;
//    public static double EXTENDO_RETRACTED = 0.0, EXTENDO_EXTENDED = 0.65;
//    public static double DOOR_OPEN = 0.5, DOOR_CLOSED = 1;
//    public static int liftZeroPos = 0;
//    public static int liftToLowBucketPos = 500;
//    public static int liftToParkPos = 250;
//    public static int liftToHighBucketPos = 1100;