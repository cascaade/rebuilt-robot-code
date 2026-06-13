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
import frc.robot.*;
import frc.robot.Constants.FieldConstants;
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

    private final MutTime lastMove;

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

        //== Required Setup ==//
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

        //== Set Defaults ==//
        rawGyroRotation = new Rotation2d();
        modulePositions = Arrays.stream(modules).map(SDSSwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, modulePositions, Constants.getInitialPose());

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

    /**
     * Sync control constants for autonomous trajectory controllers
     */
    private void syncControlConstants() {
        trajVXControllerControlConstants.applyIfChanged(trajVXController);
        trajVYControllerControlConstants.applyIfChanged(trajVYController);
        trajHeadingControllerControlConstants.applyIfChanged(trajHeadingController);
    }

    public void periodic() {
        //== State Machine ==//

        this.systemState = handleStateTransitions();
        applyStates();
        this.previousWantedState = wantedState;

        Logger.recordOutput("Swerve/WantedState", wantedState);
        Logger.recordOutput("Swerve/SystemState", systemState);

        //== Sync Control Constants ==//
        syncControlConstants();

        //== Update All Hardware Inputs ==//
        gyroIO.updateInputs(gyroIOInputs);
        Logger.processInputs("Swerve/Gyro", gyroIOInputs);

        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        SwerveModulePosition[] updatedModulePositions = new SwerveModulePosition[4];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            modules[i].periodic();

            updatedModulePositions[i] = modules[i].getPosition();
            moduleDeltas[i] = new SwerveModulePosition(
                updatedModulePositions[i].distanceMeters - modulePositions[i].distanceMeters,
                updatedModulePositions[i].angle
            );
            modulePositions[i] = updatedModulePositions[i];
            moduleStates[i] = modules[i].getCurrentState();
        }

        //== Update Raw Gyro Rotation ==//
        if (gyroIOInputs.connected) {
            rawGyroRotation = gyroIOInputs.yawPosition;
        } else {
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        //== Record All Input Data ==//
        Logger.recordOutput("Swerve/Positions", updatedModulePositions);
        Logger.recordOutput("Swerve/States/Actual", moduleStates);

        //== Update the Pose Estimator and Pose on the Dashboard ==//
        poseEstimator.update(rawGyroRotation, updatedModulePositions);
        RobotState.getInstance().addPoseObservation(poseEstimator.getEstimatedPosition());
        OperatorDashboard.getField().setRobotPose(getPose());
    }

    @AutoLogOutput(key = "Odometry/Alignment/IsAligned")
    public boolean isAligned() {
        Logger.recordOutput("Odometry/Alignment/AlignmentError", rawGyroRotation.getMeasure().minus(targetRotation.getMeasure()).abs(Degrees));
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

    /**
     * Adjust {@link OrientedChassisSpeeds} based on the need to align rotationally
     * @param speeds the speeds to align
     */
    private void adjustSpeedsForPresetRotation(OrientedChassisSpeeds speeds) {
        var robotPose = getPose();

        switch (systemState) {
            case AIMING_HUB -> {
                targetRotation = RobotState.getInstance().getFieldHubTargetHeading();
                speeds.omegaRadiansPerSecond = trajHeadingController.calculate(
                    robotPose.getRotation().getRadians(),
                    targetRotation.getRadians()
                );
            }
            case AIMING_PASS -> {
                targetRotation = RobotState.getInstance().getClosestFieldPassTargetHeading();
                speeds.omegaRadiansPerSecond = trajHeadingController.calculate(
                    robotPose.getRotation().getRadians(),
                    targetRotation.getRadians()
                );
            }
            default -> targetRotation = rawGyroRotation.rotateBy(Rotation2d.k180deg);
        }

        Logger.recordOutput("Odometry/Alignment/TargetPose", new Pose2d(robotPose.getTranslation(), targetRotation));
    }

    /**
     * Send final {@link OrientedChassisSpeeds} for this code loop to the motors
     * @param chassisSpeeds the ChassisSpeeds to send
     */
    private void submitChassisSpeeds(
        OrientedChassisSpeeds chassisSpeeds
    ) {
        //== Update lastMove ==//
        if (
            chassisSpeeds.vxMetersPerSecond != 0 ||
                chassisSpeeds.vyMetersPerSecond != 0 ||
                chassisSpeeds.omegaRadiansPerSecond != 0
        ) {
            lastMove.mut_replace(Timer.getFPGATimestamp(), Seconds);
        }

        //== Log Debug Data ==//
        Logger.recordOutput("Swerve/ChassisSpeeds/RawChassisSpeeds", chassisSpeeds.toSuper());
        Logger.recordOutput("Swerve/TimeSinceLastMove", Seconds.of(Timer.getFPGATimestamp()).minus(lastMove));

        // Set Modules to Cross if Conditions are Met ==//
        if (
            (Seconds.of(Timer.getFPGATimestamp()).minus(lastMove).gt(SwerveConstants.crossDelay))
        ) {
            setModulesToCrossPosition(true);
            return;
        }

        //== Ensure Speeds are Robot-Centric ==//
        ChassisSpeeds adjustedSpeeds = chassisSpeeds.toSuper();

        if (chassisSpeeds.fieldCentric) {
            adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds,
                getPose().getRotation().rotateBy(new Rotation2d(Constants.isRed() && !chassisSpeeds.allianceFlipped ? Math.PI : 0))
            );
        }

        //== Account for the Skew Discrete Periods Make on the Smooth Arc ==//
        adjustedSpeeds = ChassisSpeeds.discretize(adjustedSpeeds, LoggedRobot.defaultPeriodSecs);

        //== Ensure Wheel Speeds are Safe ==//
        SwerveModuleState[] moduleSetpoints = kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleSetpoints, SwerveConstants.kMaxWheelSpeed);

        //== Log States and Final ChassisSpeeds ==//
        Logger.recordOutput("Swerve/States/Setpoints", moduleSetpoints);
        Logger.recordOutput("Swerve/ChassisSpeeds/Setpoints", adjustedSpeeds);

        //== Command Setpoints of the Motors ==//
        setRawModuleSetpoints(moduleSetpoints, true);
    }

    /**
     * Set the raw setpoints (direction and speed) for each module
     * @param states the states to set
     * @param optimize whether to optimize or not (find the nearest 180deg multiple of the angle)
     */
    private void setRawModuleSetpoints(SwerveModuleState[] states, boolean optimize) {
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i], optimize);
        }
    }

    /**
     * Set the setpoints of each module to a cross position
     * @param optimize whether to optimize or not (find the nearest 180deg multiple of the angle)
     */
    private void setModulesToCrossPosition(boolean optimize) {
        setRawModuleSetpoints(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        }, optimize);
    }

    /**
     * A command builder method for zeroing the Gyro.
     *
     * Only works when connected to the FMS.
     * @return the command to zero the gyro
     */
    public Command runZeroGyro() {
        return runOnce(() -> {
            gyroIO.zeroGyro();
        })
            .andThen(new WaitCommand(0.1))
            .andThen(() -> poseEstimator.resetRotation(Constants.isRed() ? Rotation2d.kPi : Rotation2d.kZero))
            .onlyIf(() -> !DriverStation.isFMSAttached());
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public void requestFollowTrajectory(SwerveSample sample) {
        requestedSwerveSample = sample;
    }
}
