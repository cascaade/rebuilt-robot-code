package frc.robot.subsystems.swerve;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.util.*;
import frc.robot.util.SwerveMathUtil.TranslationOutput;
import lombok.Setter;
import org.littletonrobotics.junction.*;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.shooterMaxSpeed;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class SwerveFSM extends SubsystemBase {
    public enum WantedState {
        STOP,
        SYS_ID,
        AIM_HUB,
        AIM_PASS,
        TELEOP,
        TRAJECTORY,
        CROSS
    }

    private enum SystemState {
        STOPPED,
        SYS_ID,
        AIMING_HUB,
        AIMING_PASS,
        TELEOP,
        TRAJECTORY,
        CROSSED
    }

    @Setter
    private WantedState wantedState = WantedState.STOP;
    private WantedState previousWantedState = WantedState.STOP;
    private SystemState systemState = SystemState.STOPPED;

    private Rotation2d rawGyroRotation;

    private boolean toCross;
    private final MutTime lastMove;

    private final AtomicBoolean aimHubFlag;

    private PIDController trajVXController;
    private PIDController trajVYController;
    private PIDController trajHeadingController;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final CommandXboxController controller;

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroIOInputs;
    private Rotation2d targetRotation = new Rotation2d();

    private final SDSSwerveModule[] modules;
    private SwerveModulePosition[] modulePositions;

    private SwerveSample requestedSwerveSample;

    public SwerveFSM (
        CommandXboxController controller,
        GyroIO gyroIO,
        SDSModuleIO flModuleIO,
        SDSModuleIO frModuleIO,
        SDSModuleIO blModuleIO,
        SDSModuleIO brModuleIO
    ) {
        this.controller = controller;
        this.gyroIO = gyroIO;
        this.gyroIOInputs = new GyroIOInputsAutoLogged();

        toCross = true;

        modules = new SDSSwerveModule[] {
            new SDSSwerveModule("Module 0", flModuleIO),
            new SDSSwerveModule("Module 1", frModuleIO),
            new SDSSwerveModule("Module 2", blModuleIO),
            new SDSSwerveModule("Module 3", brModuleIO)
        };

        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.kWheelDistanceX.div(2), SwerveConstants.kWheelDistanceY.div(2)),
            new Translation2d(SwerveConstants.kWheelDistanceX.div(2), SwerveConstants.kWheelDistanceY.div(-2)),
            new Translation2d(SwerveConstants.kWheelDistanceX.div(-2), SwerveConstants.kWheelDistanceY.div(2)),
            new Translation2d(SwerveConstants.kWheelDistanceX.div(-2), SwerveConstants.kWheelDistanceY.div(-2))
        );

        rawGyroRotation = new Rotation2d();
        modulePositions = Arrays.stream(modules).map(SDSSwerveModule::getPosition).toArray(SwerveModulePosition[]::new);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, modulePositions, Constants.getInitialPose());

        aimHubFlag = new AtomicBoolean(false);

        trajVXController = new PIDController(0, 0, 0);
        trajVYController = new PIDController(0, 0, 0);
        trajHeadingController = new PIDController(0, 0, 0);
        trajHeadingController.enableContinuousInput(0, 2 * Math.PI);

        trajVXControllerControlConstants.applyTo(trajVXController);
        trajVYControllerControlConstants.applyTo(trajVYController);
        trajHeadingControllerControlConstants.applyTo(trajHeadingController);

        lastMove = Seconds.mutable(Timer.getFPGATimestamp());
    }

    private SystemState handleStateTransitions() {
        switch (wantedState) {
            case SYS_ID -> {
                return SystemState.SYS_ID;
            }
            case AIM_HUB -> {
                return SystemState.AIMING_HUB;
            }
            case AIM_PASS -> {
                return SystemState.AIMING_PASS;
            }
            case TELEOP -> {
                return SystemState.TELEOP;
            }
            case TRAJECTORY -> {
                return SystemState.TRAJECTORY;
            }
            case CROSS -> {
                return SystemState.CROSSED;
            }
            default -> {
                return SystemState.STOPPED;
            }
        }
    }

    private void applyStates() {
        switch (systemState) {
            case SYS_ID -> {

            }
            case AIMING_HUB -> {
                OrientedChassisSpeeds chassisSpeeds = new OrientedChassisSpeeds(
                    0,
                    0,
                    0,
                    false,
                    true
                );

                adjustSpeedsForPresetRotation(chassisSpeeds);
                submitChassisSpeeds(chassisSpeeds);
            }
            case TELEOP -> {
                double rawX = controller.getLeftX();
                double rawY = controller.getLeftY();
                double rawOmega = controller.getRightX();
                double rawSpeedFactor = controller.getLeftTriggerAxis();

                double speedFactor = SwerveMathUtil.calculateSpeedFactor(rawSpeedFactor, SwerveConstants.kSlowedMult);
                TranslationOutput translation = SwerveMathUtil.processTranslationInputs(rawX, rawY, speedFactor);
                double processedOmega = SwerveMathUtil.processRotationInput(rawOmega, speedFactor);

                OrientedChassisSpeeds chassisSpeeds = new OrientedChassisSpeeds(
                    SwerveConstants.kMagVelLimit.times(translation.x()),
                    SwerveConstants.kMagVelLimit.times(translation.y()),
                    SwerveConstants.kRotVelLimit.times(processedOmega),
                    false,
                    true
                );

                adjustSpeedsForPresetRotation(chassisSpeeds);
                submitChassisSpeeds(chassisSpeeds);
            }
            case AIMING_PASS -> {
                double rawX = controller.getLeftX();
                double rawY = controller.getLeftY();
                double rawOmega = controller.getRightX();
                double rawSpeedFactor = controller.getLeftTriggerAxis();

                double speedFactor = SwerveMathUtil.calculateSpeedFactor(rawSpeedFactor, SwerveConstants.kSlowedMult) * 0.8;
                TranslationOutput translation = SwerveMathUtil.processTranslationInputs(rawX, rawY, speedFactor);
                double processedOmega = SwerveMathUtil.processRotationInput(rawOmega, speedFactor);

                OrientedChassisSpeeds chassisSpeeds = new OrientedChassisSpeeds(
                    SwerveConstants.kMagVelLimit.times(translation.x()),
                    SwerveConstants.kMagVelLimit.times(translation.y()),
                    SwerveConstants.kRotVelLimit.times(processedOmega),
                    false,
                    true
                );

                adjustSpeedsForPresetRotation(chassisSpeeds);
                submitChassisSpeeds(chassisSpeeds);
            }
            case TRAJECTORY -> {
                if (requestedSwerveSample == null) {
                    for (int i = 0; i < 4; i++) {
                        modules[i].stopDrive();
                    }
                    return;
                }

                Pose2d currentPose = getPose();

                OrientedChassisSpeeds speeds = new OrientedChassisSpeeds(
                    requestedSwerveSample.vx + trajVXController.calculate(currentPose.getX(), requestedSwerveSample.x),
                    requestedSwerveSample.vy + trajVYController.calculate(currentPose.getY(), requestedSwerveSample.y),
                    requestedSwerveSample.omega + trajHeadingController.calculate(
                        currentPose.getRotation().getRadians(),
                        requestedSwerveSample.heading
                    ),
                    true, true
                );

                Logger.recordOutput("Swerve/ChassisSpeeds/Auto", speeds);

                adjustSpeedsForPresetRotation(speeds);
                submitChassisSpeeds(speeds);

                // dispose of swerve sample to prevent repetition
                requestedSwerveSample = null;
            }
            case STOPPED -> {
                for (int i = 0; i < 4; i++) {
                    modules[i].stopDrive();
                }
            }
            case CROSSED -> {
                setModulesToCrossPosition(false);
            }
            default -> {

            }
        }
    }

    public void periodic() {
        this.systemState = handleStateTransitions();
        applyStates();
        this.previousWantedState = wantedState;

        Logger.recordOutput("Swerve/WantedState", wantedState);
        Logger.recordOutput("Swerve/SystemState", systemState);

        trajVXControllerControlConstants.applyIfChanged(trajVXController);
        trajVYControllerControlConstants.applyIfChanged(trajVYController);
        trajHeadingControllerControlConstants.applyIfChanged(trajHeadingController);

        Logger.recordOutput("Swerve/AimHubFlag", aimHubFlag.get());
        Logger.recordOutput("Swerve/CrossEnabled", toCross);

        // updated all hardware inputs
        gyroIO.updateInputs(gyroIOInputs);
        Logger.processInputs("Swerve/Gyro", gyroIOInputs);

        for (SDSSwerveModule module : modules) {
            module.periodic();
        }

        SwerveModuleState[] moduleStates = new SwerveModuleState[4];

        // process updates from hardware
        SwerveModulePosition[] updatedModulePositions = new SwerveModulePosition[4];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            updatedModulePositions[i] = modules[i].getPosition();
            moduleDeltas[i] = new SwerveModulePosition(
                updatedModulePositions[i].distanceMeters - modulePositions[i].distanceMeters,
                updatedModulePositions[i].angle
            );
            modulePositions[i] = updatedModulePositions[i];
            moduleStates[i] = modules[i].getCurrentState();
        }

        if (gyroIOInputs.connected) {
            rawGyroRotation = gyroIOInputs.yawPosition;
        } else {
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // record updated positions and update odometry
        Logger.recordOutput("Swerve/Positions", updatedModulePositions);
        Logger.recordOutput("Swerve/States/Actual", moduleStates);
        poseEstimator.update(rawGyroRotation, updatedModulePositions);

        OperatorDashboard.getField().setRobotPose(getPose());

        if (DriverStation.isDisabled()) {
            aimHubFlag.set(false);
        }

        RobotState.getInstance().addPoseObservation(poseEstimator.getEstimatedPosition());

        Logger.recordOutput("Swerve/Drive_Command", this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName());
    }

    @AutoLogOutput(key = "Swerve/IsAligned")
    public boolean isAligned() {
        Logger.recordOutput("Swerve/AlignmentError", rawGyroRotation.getMeasure().minus(targetRotation.getMeasure()).abs(Degrees));
        return rawGyroRotation.getMeasure().isNear(targetRotation.getMeasure(), BODY_ROTATION_ALIGN_TOLERANCE);
    }

    @AutoLogOutput(key = "Odometry/Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp, Matrix<N3, N1> stdDevs) {
        // higher standard deviations means vision measurements are trusted less
        poseEstimator.addVisionMeasurement(visionMeasurement, timestamp, stdDevs);
        getPose();
    }

    private void adjustSpeedsForPresetRotation(OrientedChassisSpeeds speeds) {
        Pose2d robotPose = getPose();
        Pose2d hubPose = FieldConstants.getHubCenter();

        Translation2d robotToHub = hubPose.getTranslation().minus(robotPose.getTranslation());
        Rotation2d targetHeading = robotToHub.getAngle().minus(Rotation2d.k180deg);

        Logger.recordOutput("Swerve/AutoAlignTargetPose", new Pose2d(robotPose.getTranslation(), targetHeading));
        Logger.recordOutput("Swerve/DistanceToHub", robotToHub.getNorm());
        Logger.recordOutput("Field/HubPose", hubPose);
        Logger.recordOutput("Swerve/AutoAlignUpdate", Timer.getFPGATimestamp());

        switch (systemState) {
            case AIMING_HUB -> {
                speeds.omegaRadiansPerSecond = trajHeadingController.calculate(
                    robotPose.getRotation().getRadians(),
                    targetHeading.getRadians()
                );
                targetRotation = targetHeading;
            }
            case AIMING_PASS -> {
                speeds.omegaRadiansPerSecond = trajHeadingController.calculate(
                    robotPose.getRotation().getRadians(),
                    RobotState.getInstance().getClosestFieldPassTargetHeading().getRadians()
                );
                targetRotation = RobotState.getInstance().getClosestFieldPassTargetHeading();
            }
            default -> targetRotation = rawGyroRotation.rotateBy(Rotation2d.k180deg);
        }

        Logger.recordOutput("Swerve/targetRotation", new Pose2d(robotPose.getTranslation(), targetRotation));
    }

    private void submitChassisSpeeds(
        OrientedChassisSpeeds chassisSpeeds
    ) {

        // Update lastMove
        if (
            chassisSpeeds.vxMetersPerSecond != 0 ||
                chassisSpeeds.vyMetersPerSecond != 0 ||
                chassisSpeeds.omegaRadiansPerSecond != 0
        ) {
            lastMove.mut_replace(Timer.getFPGATimestamp(), Seconds);
        }

        Logger.recordOutput("Swerve/ChassisSpeeds/RawChassisSpeeds", new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond));
//        Logger.recordOutput("Swerve/ChassisSpeeds/rawVelocity", chassisSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput("Swerve/timeSinceLastMove", Seconds.of(Timer.getFPGATimestamp()).minus(lastMove));

        // Set modules to cross if the time since last move exceeds threshold
        if (
            (toCross && Seconds.of(Timer.getFPGATimestamp()).minus(lastMove).gt(SwerveConstants.crossDelay))
        ) {
            setModulesToCrossPosition(true);
            return;
        }

        // Adjust speeds to be robot centric if they aren't already
        ChassisSpeeds adjustedSpeeds = chassisSpeeds;

        if (chassisSpeeds.fieldCentric) {
            adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds,
                getPose().getRotation().rotateBy(new Rotation2d(Constants.isRed() && !chassisSpeeds.allianceFlipped ? Math.PI : 0))
            );
        }

        // Account for the skew discrete periods make on the smooth arc
        adjustedSpeeds = ChassisSpeeds.discretize(adjustedSpeeds, LoggedRobot.defaultPeriodSecs);

        SwerveModuleState[] moduleSetpoints = kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleSetpoints, SwerveConstants.kMaxWheelSpeed);

        Logger.recordOutput("Swerve/States/Setpoints", moduleSetpoints);
        Logger.recordOutput("Swerve/ChassisSpeeds/Setpoints", adjustedSpeeds);

        setRawModuleSetpoints(moduleSetpoints, true);
    }

    private void setRawModuleSetpoints(SwerveModuleState[] states, boolean optimize) {
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i], optimize);
        }
    }

    private void setModulesToCrossPosition(boolean optimize) {
        setRawModuleSetpoints(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        }, optimize);
    }

    public Command runZeroGyro() {
        if (!DriverStation.isFMSAttached()) {
            return runOnce(() -> {
                gyroIO.zeroGyro();
            })
                .andThen(new WaitCommand(0.1))
                .andThen(() -> {
                    poseEstimator.resetRotation(Constants.isRed() ? Rotation2d.kPi : Rotation2d.kZero);
                });
        }
        return Commands.none();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public void requestFollowTrajectory(SwerveSample sample) {
        requestedSwerveSample = sample;
    }
}
