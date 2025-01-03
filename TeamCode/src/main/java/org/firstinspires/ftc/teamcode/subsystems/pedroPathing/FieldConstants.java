package org.firstinspires.ftc.teamcode.subsystems.pedroPathing;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.localization.Pose;

@Config
public class FieldConstants {

    public enum RobotStart {
        BLUE_BUCKET,
        BLUE_OBSERVATION,
        RED_BUCKET,
        RED_OBSERVATION
    }

    public static final Pose blueBucketStartPose = new Pose(0,0, Math.toRadians(0));;
    public static final Pose blueObservationStartPose = new Pose(8, 36, Math.toRadians(180));
    public static final Pose redBucketStartPose = new Pose(144-8, 79.5, 0);
    public static final Pose redObservationStartPose = new Pose(144-8, 36, 0);

    // Blue Preload Poses
    public static final Pose blueBucketPreloadPose = new Pose(6.65,13.25, Math.toRadians(-45));

    // Blue Bucket Sample Poses
    public static final Pose blueBucketLeftSamplePose = new Pose(11,15, Math.toRadians(25));
//    public static final Pose blueBucketLeftSampleControlPose = new Pose(32, 108);
    public static final Pose blueBucketMidSamplePose =  new Pose(11,14, Math.toRadians(0));
//    public static final Pose blueBucketMidSampleControlPose = new Pose(11,15, Math.toRadians(25));
    public static final Pose blueBucketRightSamplePose = new Pose(10.885,11.3, Math.toRadians(0));
//    public static final Pose blueBucketRightSampleControlPose = new Pose(46, 101);

    public static final Pose blueBucketScorePose = new Pose(8,13.5, Math.toRadians(-45));

    public static final Pose blueBucketParkPose = new Pose(50,-10.7 , Math.toRadians(90));
    public static final Pose blueBucketParkControlPose = new Pose(60, 4, Math.toRadians(90));


    public static Pose startPose = new Pose(0,0, Math.toRadians(0));
    public static Pose sample1Pos = new Pose(11,19.5, Math.toRadians(0));
    public static Pose sample2Pos = new Pose(10.885,11.3, Math.toRadians(0));
    public static Pose sample3Pos = new Pose(11,15, Math.toRadians(25));

    public static Pose basketPos = new Pose(6.65,13.25, Math.toRadians(-45));


}