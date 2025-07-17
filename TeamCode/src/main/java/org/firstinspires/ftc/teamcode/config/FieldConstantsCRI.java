package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.localization.Pose;


@Config
public class FieldConstantsCRI {


//    public static double WALLDIST = 12.3, BARDIST = 39.5, BARDIST2 = 40;

    public enum RobotStart {
        BLUE_BUCKET,
        BLUE_OBSERVATION,
        RED_BUCKET,
        RED_OBSERVATION
    }
    public static final Pose blueBucketStartPose = new Pose(0,0, Math.toRadians(0));;
    public static final Pose blueObservationStartPose = new Pose(7.5, 65.25, Math.toRadians(0));

    // Preload Poses
    public static final Pose blueBucketPreloadPose = new Pose(7.7,-15.1, Math.toRadians(45));
    public static final Pose blueObservationPreloadPose = new Pose(40.5, 75, Math.toRadians(0));

    // Blue Bucket Sample Poses
    public static final Pose blueBucketLeftSamplePose = new Pose(11.42,-12, Math.toRadians(32));
    public static final Pose redBucketLeftSamplePose = new Pose(-11.42,12, Math.toRadians(32));

    //    public static final Pose blueBucketLeftSampleControlPose = new Pose(32, 108);
    public static final Pose blueBucketMidSamplePose =  new Pose(9.3,-15.5, Math.toRadians(-1));
//    public static final Pose redBucketMidSamplePose =  new Pose(-9.3,15.5, Math.toRadians(-1));

    //    public static final Pose blueBucketMidSampleControlPose = new Pose(11,15, Math.toRadians(25));
    public static final Pose blueBucketRightSamplePose = new Pose(8.7,-11, Math.toRadians(0));
    public static final Pose redBucketRightSamplePose = new Pose(-8.7,11, Math.toRadians(0));

    //    public static final Pose blueBucketRightSampleControlPose = new Pose(46, 101);

    public static final Pose blueBucketScorePose = new Pose(7,-16, Math.toRadians(45));
    public static final Pose redBucketScorePose = new Pose(-7,16, Math.toRadians(-45));


//    public static final Pose blueObservationPushingEndPose = new Pose(22.5, 68.0, Math.toRadians(0));


    //CRI poses
    public static final Pose commonAreaBlockPose = new Pose(-75, 0, Math.toRadians(0));
    public static final Pose commonAreaPreScorePose = new Pose(-98, 0, Math.toRadians(0));
    public static final Pose oppositeAreaControlBucketScorePose = new Pose(-100, -22, Math.toRadians(0));
    public static final Pose oppositeAreaBucketScorePose = new Pose(-117, -17.6, Math.toRadians(45));


    // Blue Observation Specimen Poses
    public static final Pose blueObservationSpecimenSetPose = new Pose(13, 35, Math.toRadians(0));
    public static final Pose blueObservationSpecimenPickupPose = new Pose(11, 12, Math.toRadians(0));
    public static final Pose blueObservationSpecimenPickup2Pose = new Pose(11, 36, Math.toRadians(0));
    public static final Pose blueObservationSpecimenPickup3Pose = new Pose(11, 36, Math.toRadians(0));
    public static final Pose blueObservationSpecimenPickup4Pose = new Pose(11, 36, Math.toRadians(0));
    public static final Pose blueObservationSpecimen1Pose = new Pose(42.25, 70.1, Math.toRadians(0));
    public static final Pose blueObservationSpecimen2Pose = new Pose(42.25, 69.4, Math.toRadians(0));
    public static final Pose blueObservationSpecimen3Pose = new Pose(42.5, 68.7, Math.toRadians(0));
    public static final Pose blueObservationSpecimen4Pose = new Pose(42.5, 68.0, Math.toRadians(0));


    // Park Poses
    public static final Pose blueBucketParkPose = new Pose(54,-14 , Math.toRadians(90));
    public static final Pose blueBucketParkControlPose = new Pose(60, 4, Math.toRadians(90));

    public static final Pose blueObservationParkPose = new Pose(12, 32, Math.toRadians(0));

    public static final Pose blueBucketObsPark = new Pose(85.5213,22, 0);




    public static final Pose specStart = new Pose(7.5, 65.25, Math.toRadians(0));
    public static final Pose specScoring = new Pose(42.5, 69, Math.toRadians(0));
    public static final Pose specPickup = new Pose(10, 38.5, Math.toRadians(0));
    public static final Pose specIntakeAvoid = new Pose(28, 45, Math.toRadians(-90));
    public static final Pose specIntake1 = new Pose(30, 40, Math.toRadians(-40));
    public static final Pose specDrop1 = new Pose(30, 30, Math.toRadians(-100));
    public static final Pose specIntake2 = new Pose(44, 30, Math.toRadians(-90));
    public static final Pose specDrop2 = new Pose(30, 25, Math.toRadians(-100));
    public static final Pose specIntake3 = new Pose(43.5, 28, Math.toRadians(-90));
    public static final Pose specDrop3 = new Pose(30, 20, Math.toRadians(-100));
    public static final Pose specAvoidPickup = new Pose(24, 16, Math.toRadians(0));
    public static final Pose specPark = new Pose(12, 32, Math.toRadians(-90));


}