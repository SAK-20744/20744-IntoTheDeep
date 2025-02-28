package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.WALLDIST;

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

    public static final Pose blueBucketStartPose = new Pose(7.5, 78.75, Math.toRadians(0));
    public static final Pose blueObservationStartPose = new Pose(7.5, 65.25, Math.toRadians(0));

    // Preload Poses
    public static final Pose blueBucketPreloadPose = new Pose(29.25, 69, Math.toRadians(0));
    public static final Pose blueObservationPreloadPose = new Pose(38.5, 75, Math.toRadians(0));

    // Blue Bucket Sample Poses
    public static final Pose blueBucketLeftSamplePose = new Pose(22, 110, 0);
    public static final Pose blueBucketLeftSampleControlPose = new Pose(20, 96);
    public static final Pose blueBucketMidSamplePose = new Pose(22, 116, 0);
    public static final Pose blueBucketMidSampleControlPose = new Pose(20, 86);
    public static final Pose blueBucketRightSamplePose = new Pose(22, 122, 0);
    public static final Pose blueBucketRightSampleControlPose = new Pose(20, 96);
    public static final Pose blueBucketScorePose = new Pose(20, 128, Math.toRadians(-45));

    // Blue Observation Specimen Poses
    public static final Pose blueObservationSpecimenSetPose = new Pose(12, 35, Math.toRadians(0));
    public static final Pose blueObservationSpecimenPickupPose = new Pose(10.8, 12, Math.toRadians(0));
    public static final Pose blueObservationSpecimenPickup2Pose = new Pose(WALLDIST, 36, Math.toRadians(0));
    public static final Pose blueObservationSpecimenPickup3Pose = new Pose(WALLDIST, 36, Math.toRadians(0));
    public static final Pose blueObservationSpecimenPickup4Pose = new Pose(WALLDIST, 36, Math.toRadians(0));
    public static final Pose blueObservationSpecimen1Pose = new Pose(40.5, 70.1, Math.toRadians(0));
    public static final Pose blueObservationSpecimen2Pose = new Pose(40.5, 69.4, Math.toRadians(0));
    public static final Pose blueObservationSpecimen3Pose = new Pose(40.8, 68.7, Math.toRadians(0));
    public static final Pose blueObservationSpecimen4Pose = new Pose(40.8, 68.0, Math.toRadians(0));

    // Park Poses
    public static final Pose blueBucketParkPose = new Pose(62, 97.75, Math.toRadians(90));
    public static final Pose blueBucketParkControlPose = new Pose(60.25, 123.5);
    public static final Pose blueObservationParkPose = new Pose(12, 32, Math.toRadians(0));


}