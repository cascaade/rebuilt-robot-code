package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;
import lombok.Getter;

import static edu.wpi.first.units.Units.Meters;

public class RobotState {
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    @Getter
    private Pose2d robotFieldPose = new Pose2d();

    public void addPoseObservation(Pose2d robotPose) {
        this.robotFieldPose = robotPose;
    }

    public Distance getFieldHubDistance() {
        Pose2d hubPose = FieldConstants.getHubCenter();

        return Meters.of(robotFieldPose.getTranslation().getDistance(hubPose.getTranslation()));
    }

    public Rotation2d getFieldHubTargetHeading() {
        Pose2d hubPose = FieldConstants.getHubCenter();

        Translation2d robotToHub = hubPose.getTranslation().minus(robotFieldPose.getTranslation());
        return robotToHub.getAngle().minus(Rotation2d.k180deg);
    }

    public Distance getClosestFieldPassDistance() {
        boolean shouldPassLow = getRobotFieldPose().getMeasureY().gt(FieldConstants.getHubCenter().getMeasureY());
        Pose2d passPose = shouldPassLow ? FieldConstants.getPassLow() : FieldConstants.getPassHigh();

        return Meters.of(robotFieldPose.getTranslation().getDistance(passPose.getTranslation()));
    }

    public Rotation2d getClosestFieldPassTargetHeading() {
        boolean shouldPassLow = getRobotFieldPose().getMeasureY().gt(FieldConstants.getHubCenter().getMeasureY());
        Pose2d passPose = shouldPassLow ? FieldConstants.getPassLow() : FieldConstants.getPassHigh();

        Translation2d robotToPass = passPose.getTranslation().minus(robotFieldPose.getTranslation());
        return robotToPass.getAngle().minus(Rotation2d.k180deg);
    }

    public boolean isOutsideAllianceZone() {
        return Constants.isRed() ?
            getRobotFieldPose().getMeasureX().lt(FieldConstants.getHubCenter().getMeasureX())
            : getRobotFieldPose().getMeasureX().gt(FieldConstants.getHubCenter().getMeasureX());
    }
}
