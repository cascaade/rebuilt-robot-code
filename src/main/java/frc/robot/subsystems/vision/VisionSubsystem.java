package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    public final VisionConsumer consumer;
    public final VisionIO[] ios;
    public final VisionIOInputsAutoLogged[] inputs;

    // private final LoggedNetworkBoolean recordingRequest = new LoggedNetworkBoolean("SmartDashboard/Enable Recording", false);

    public VisionSubsystem(VisionConsumer consumer, VisionIO... ios) {
        this.consumer = consumer;
        this.ios = ios;
        this.inputs = new VisionIOInputsAutoLogged[ios.length];

        for (int i = 0; i < ios.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    private boolean shouldRejectPose(PoseObservation observation) {
        return observation.tagCount() == 0 // reject if no tags available
            || (observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.maxAmbiguity) // reject if above max ambiguity
            || Math.abs(observation.pose().getZ()) > VisionConstants.maxZError // reject if Z is above error threshold

            // reject if out of field perimeter
            || observation.pose().getX() < 0.0
            || observation.pose().getX() > VisionConstants.aprilTagLayout.getFieldLength()
            || observation.pose().getY() < 0.0
            || observation.pose().getY() > VisionConstants.aprilTagLayout.getFieldWidth();
    }

    private record VisionStdDevs(double linear, double angular) {}

    private VisionStdDevs calculateStandardDeviations(
        int cameraIndex,
        PoseObservation observation
    ) {
        double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
        double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;
        
        if (observation.type() == PoseObservationType.MEGATAG_2) {
            linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
            angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
        }

        if (cameraIndex < VisionConstants.camConfigs.length) {
            linearStdDev *= VisionConstants.camConfigs[cameraIndex].standardDeviationMultiplier();
            angularStdDev *= VisionConstants.camConfigs[cameraIndex].standardDeviationMultiplier();
        }

        return new VisionStdDevs(linearStdDev, angularStdDev);
    }

    private void processCameraPoseObservation(
        int cameraIndex,
        PoseObservation observation,
        List<Pose3d> tagPoses,
        List<Pose3d> robotPoses,
        List<Pose3d> robotPosesAccepted,
        List<Pose3d> robotPosesRejected
    ) {
        // Check whether to reject pose
        boolean rejectPose = shouldRejectPose(observation);

        robotPoses.add(observation.pose());
        if (rejectPose) {
            robotPosesRejected.add(observation.pose());
        } else {
            robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) return;

        // calculate standard deviations
        VisionStdDevs standardDeviations = calculateStandardDeviations(cameraIndex, observation);

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(standardDeviations.linear(), standardDeviations.linear(), standardDeviations.angular())
        );
    }

    private void processAllCameraOutputs(
        int cameraIndex,
        List<Pose3d> allTagPoses,
        List<Pose3d> allRobotPoses,
        List<Pose3d> allRobotPosesAccepted,
        List<Pose3d> allRobotPosesRejected
    ) {
        // Initialize logging values
        List<Pose3d> tagPoses = new ArrayList<>();
        List<Pose3d> robotPoses = new ArrayList<>();
        List<Pose3d> robotPosesAccepted = new ArrayList<>();
        List<Pose3d> robotPosesRejected = new ArrayList<>();

        // Add tag poses
        for (int tagId : inputs[cameraIndex].tagIds) {
            var tagPose = VisionConstants.aprilTagLayout.getTagPose(tagId);
            if (tagPose.isPresent()) {
                tagPoses.add(tagPose.get());
            }
        }

        // Loop over pose observations
        for (PoseObservation observation : inputs[cameraIndex].poseObservations) {
            processCameraPoseObservation(
                cameraIndex,
                observation,
                tagPoses, 
                robotPoses,
                robotPosesAccepted,
                robotPosesRejected
            );
        }

        // Log camera metadata
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
            tagPoses.toArray(new Pose3d[0])
        );
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
            robotPoses.toArray(new Pose3d[0])
        );
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
            robotPosesAccepted.toArray(new Pose3d[0])
        );
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
            robotPosesRejected.toArray(new Pose3d[0])
        );
        
        allTagPoses.addAll(tagPoses);
        allRobotPoses.addAll(robotPoses);
        allRobotPosesAccepted.addAll(robotPosesAccepted);
        allRobotPosesRejected.addAll(robotPosesRejected);
    }

    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Inst" + i, inputs[i]);
        }

        List<Pose3d> allTagPoses = new ArrayList<>();
        List<Pose3d> allRobotPoses = new ArrayList<>();
        List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
        List<Pose3d> allRobotPosesRejected = new ArrayList<>();

        // boolean shouldRecord = DriverStation.isFMSAttached() || recordingRequest.get();

        for (int cameraIndex = 0; cameraIndex < ios.length; cameraIndex++) {
            processAllCameraOutputs(
                cameraIndex,
                allTagPoses,
                allRobotPoses,
                allRobotPosesAccepted,
                allRobotPosesRejected
            );
        }

        // Log summary data
        Logger.recordOutput(
            "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0])
        );
        Logger.recordOutput(
            "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0])
        );
        Logger.recordOutput(
            "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0])
        );
        Logger.recordOutput(
            "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0])
        );
    }


    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
        );
    }
}
