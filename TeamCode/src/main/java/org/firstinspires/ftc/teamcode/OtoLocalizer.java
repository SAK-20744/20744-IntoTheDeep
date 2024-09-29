package org.firstinspires.ftc.teamcode;

public class OtoLocalizer implements Localizer {

    public final SparkFunOTOS sensor;

    public OtoLocalizer(SparkFunOTOS sensor) {
        this.sensor = sensor;
    }

    @Override
    public Pose2d getPoseEstimate() {
        SparkFunOTOS.Pose2D pose = sensor.getPosition();
        return new Pose2d(pose.x, pose.y, Angle.rad(pose.h));
    }

    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {}
}
