package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    public static double
            INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0.75,
            LDIFFY_TRANSFERING = 0.1, LDIFFY_SCORING = 0.475, LDIFFY_DROP = 0.5,
            RDIFFY_TRANSFERING = 0.1, RDIFFY_SCORING = 0.475, RDIFFY_DROP = 0.5,
            CLAW_CLOSED = 0.28, CLAW_OPEN = 0.65, CLAW_SPEC = 0.38, CLAW_INT = 0.3105,
            WRIST_TRANSFERING = 0.93, WRIST_UP = 0.625, WRIST_INTAKING = 0.205, WRIST_CLOSE_INTAKING = 0.18,
            E_RETRACT_POWER = -0.5,
            redVal = 0.035, blueVal = 0.035, greenVal = 0.5,
            wristNeeded = 145;

    public static int
            LIFT_RETRACTED = -7, LIFT_MID_BASKET = 500, LIFT_HIGH_BASKET = 1350, LIFT_HIGH_RUNG = 410, LIFT_HIGH_RUNG_ADJUSTED = 458,  LIFT_AUTO_RUNG = 378, LIFT_MID_RUNG = 300, clipRange = 20,
            EXTENDO_RETRACTED = -25, EXTENDO_RETRACTED_TELE = -250, EXTENDO_EXTENDED = 500, EXTENDO_AUTO = 220, EXTENDO_OUTTAKE = 275,
            RANGEFINDERRANGE = 65;

    public static double lp = -0.0085, li = 0.000275, ld = 0;
    public static double autoP = -0.01, autoI = 0.007, autoD = 0.0000;
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