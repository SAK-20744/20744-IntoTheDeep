package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    public static double
            INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0.3,
            LDIFFY_TRANSFERING = 0.86, LDIFFY_CLIPPING = 0.42, LDIFFY_SCORING = 0.65, LDIFFY_WALL = 0.93,
            RDIFFY_TRANSFERING = 0.87, RDIFFY_CLIPPING = 0, RDIFFY_SCORING = 0.5, RDIFFY_WALL = 0.82,
            CLAW_CLOSED = 0.55, CLAW_OPEN = 0.25,
            WRIST_TRANSFERING = 0.9, WRIST_UP = 0.5, WRIST_INTAKING = 0.2,
            DOOR_OPEN = 0.6, DOOR_CLOSED = 0.2,
            ROLL_DEPO = 0.55, ROLL_TRANSFERING = 0,
            RAIL_TRANSFERING = 0.2, RAIL_WALL= 0.5, RAIL_SCORING = 0, RAIL_CLIPPING = 1,
            E_RETRACT_POWER = -0;

    public static int
            LIFT_RETRACTED = -5, LIFT_MID_BASKET = 500, LIFT_HIGH_BASKET = 1250, LIFT_HIGH_RUNG = 450, LIFT_MID_RUNG = 300, clipRange = 250,
            EXTENDO_RETRACTED = -25, EXTENDO_EXTENDED = 450, EXTENDO_AUTO = 250;

    public static double WALLDIST = 13.25;

    public static double lp = -0.0088, li = 0.07, ld = 0.000000012;
    public static double ep = 0.038, ei = 0, ed = 0.000005;

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