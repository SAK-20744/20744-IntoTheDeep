package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    public enum Side {
        LEFT,
        RIGHT
    }

    public static int LEFTSIDE_REGION_X = 0;
    public static int LEFTSIDE_REGION_Y = 170;

    public static int RIGHTSIDE_REGION_X = 0;
    public static int RIGHTSIDE_REGION_Y = 170;

    public static int REGION_WIDTH = 65;
    public static int REGION_HEIGHT = 65;

    public static boolean reached = false;

    // Lift Subsystem Constants
    public static int HIGH_BASKET_POS = 597;
    public static int HIGH_BAR_POS = 365;
    public static int MID_BASKET_POS = 597;
    public static int MID_BAR_POS = 365;
    public static int LIFT_RETRACT_POS = -5;

    public static double CLAW_OPEN = 0;
    public static double CLAW_CLOSED = 0.5;

    // in/s and in/s^2
    public static double LIFT_MAX_V = 100;
    public static double LIFT_MAX_A = 99999;
    public static double LIFT_MAX_A_RETRACT = 250;
    public static double LIFT_MAX_D = 250;

    public static final double LIFT_TICKS_PER_INCH = 22.754;

    public static double LIFT_MANUAL_FACTOR = 1;
    public static double LIFT_MIN = 0;
    public static double LIFT_MAX = 0;

    public static double LIFT_EXTENDED_TOLERANCE = 20;
    public static double LIFT_ERROR_TOLERANCE = 20;

    // Intake Subsystem Constants
    public static double INTAKE_EXTENSION_MAX_V = 50;
    public static double INTAKE_EXTENSION_MAX_A = 50;
    public static double INTAKE_EXTENSION_MAX_D = 50;

    public static long INTAKE_CLAW_CLOSE_TIME = 150; // ms

    public static final double EXTENSION_TICKS_PER_INCH = 23.54;

    public static Side SIDE = Side.LEFT;
    public static boolean AUTO = false;
    public static boolean USING_IMU = true;
    public static boolean MANUAL_ENABLED = true;
    public static boolean HAS_AUTO_ERROR = false;
    public static boolean IS_PARKING = false;

    public static int wait1 = 100;
    public static int wait2 = 300;
    public static int wait3 = 50;
    public static int wait4 = 50;
    public static int wait5 = 200;
    public static int wait6 = 500;
}