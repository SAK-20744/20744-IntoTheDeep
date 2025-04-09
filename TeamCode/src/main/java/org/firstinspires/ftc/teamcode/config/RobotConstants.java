package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    public static double
            INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0.5,
            LDIFFY_TRANSFERING = 0.94, LDIFFY_CLIPPING = 0.625, LDIFFY_SCORING = 0.49, LDIFFY_WALL = 0.16, LDIFFY_AUTO = 0.4,
            RDIFFY_TRANSFERING = 0.83, RDIFFY_CLIPPING = 1, RDIFFY_SCORING = 0.45, RDIFFY_WALL = 0.2, RDIFFY_AUTO = 0.35,
            CLAW_CLOSED = 0.1, CLAW_OPEN = 0.38,
            WRIST_TRANSFERING = 1, WRIST_UP = 0.5785, WRIST_INTAKING = 0.342, WRIST_CLOSE_INTAKING = 0.23,
            DOOR_OPEN = 0.6, DOOR_CLOSED = 0.2,
            ROLL_DEPO = 0.75, ROLL_TRANSFERING = 0.16, YAW = 0.125,
            LRAIL_TRANSFERING = 0.975, LRAIL_WALL= 0.825, LRAIL_SCORING = 1, LRAIL_CLIPPING = 0,
            RRAIL_TRANSFERING = 0.975, RRAIL_WALL= 0.825, RRAIL_SCORING = 1, RRAIL_CLIPPING = 0,
            E_RETRACT_POWER = -0.15,
            redVal = 0.02, blueVal = 0.02, greenVal = 0.02,
            grabtime = 0.3;

    public static int
            LIFT_RETRACTED = -2, LIFT_MID_BASKET = 500, LIFT_HIGH_BASKET = 1350, LIFT_HIGH_RUNG = 440, LIFT_HIGH_RUNG_ADJUSTED = 458,  LIFT_AUTO_RUNG = 450, LIFT_MID_RUNG = 300, clipRange = 20,
            EXTENDO_RETRACTED = -25, EXTENDO_EXTENDED = 450, EXTENDO_AUTO = 220, EXTENDO_OUTTAKE = 275,
            RANGEFINDERRANGE = 65;

    public static double lp = -0.0085, li = 0.000275, ld = 0;
    public static double autoP = -0.01, autoI = 0.0175, autoD = 0;
    public static double ep = 0.02, ei = 0, ed = 0.000005;

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