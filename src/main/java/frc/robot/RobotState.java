package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;
import lombok.Getter;
import org.littletonrobotics.junction.*;

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

    /**
     * Update the RobotState copy of the robot's position on the field
     * @param robotPose the pose to set
     */
    public void addPoseObservation(Pose2d robotPose) {
        this.robotFieldPose = robotPose;
    }

    /**
     *
     * @return the distance from the robot to the scoring hub
     */
    @AutoLogOutput(key = "Odometry/Alignment/HubDistance")
    public Distance getFieldHubDistance() {
        Pose2d hubPose = FieldConstants.getHubCenter();

        return Meters.of(robotFieldPose.getTranslation().getDistance(hubPose.getTranslation()));
    }

    /**
     *
     * @return the field-oriented heading needed to align the robot to the hub
     */
    @AutoLogOutput(key = "Odometry/Alignment/HubTargetHeading")
    public Rotation2d getFieldHubTargetHeading() {
        Pose2d hubPose = FieldConstants.getHubCenter();

        Translation2d robotToHub = hubPose.getTranslation().minus(robotFieldPose.getTranslation());
        return robotToHub.getAngle().minus(Rotation2d.k180deg);
    }

    /**
     *
     * @return true if the robot should pass toward the right half of the field, otherwise false
     */
    @AutoLogOutput(key = "Odometry/ShouldPassRight")
    public boolean shouldPassRight() {
        boolean blue = getRobotFieldPose().getMeasureY().lt(FieldConstants.getHubCenter().getMeasureY());
        return Constants.isRed() != blue; // XOR - reference truth tables
    }

    /**
     *
     * @return distance to the closest pass point
     */
    @AutoLogOutput(key = "Odometry/Alignment/ClosestPassDistance")
    public Distance getClosestFieldPassDistance() {
        Pose2d passPose = shouldPassRight() ? FieldConstants.getPassLow() : FieldConstants.getPassHigh();

        return Meters.of(robotFieldPose.getTranslation().getDistance(passPose.getTranslation()));
    }

    /**
     *
     * @return the field-oriented heading needed to align the robot to the closest pass point
     */
    @AutoLogOutput(key = "Odometry/Alignment/ClosestPassTargetHeading")
    public Rotation2d getClosestFieldPassTargetHeading() {
        Pose2d passPose = shouldPassRight() ? FieldConstants.getPassLow() : FieldConstants.getPassHigh();

        Translation2d robotToPass = passPose.getTranslation().minus(robotFieldPose.getTranslation());
        return robotToPass.getAngle().minus(Rotation2d.k180deg);
    }

    @AutoLogOutput(key = "Odometry/OutsideAllianceZone")
    public boolean isOutsideAllianceZone() {
        return Constants.isRed() ?
            getRobotFieldPose().getMeasureX().lt(FieldConstants.getHubCenter().getMeasureX())
            : getRobotFieldPose().getMeasureX().gt(FieldConstants.getHubCenter().getMeasureX());
    }
}
