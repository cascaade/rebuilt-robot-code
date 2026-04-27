package frc.robot.subsystems.swerve;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Seconds;

public class SwerveDrive extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroIOInputs;
    
    private final SDSSwerveModule[] modules;
    private SwerveModulePosition[] modulePositions;
    
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private boolean toCrossbuck;
    private final MutTime lastMove;

    private final AtomicBoolean aimHubFlag;

    private PIDController trajVXController;
    private PIDController trajVYController;
    private PIDController trajHeadingController;

    private Rotation2d rawGyroRotation;

    private final Field2d field;

    public SwerveDrive(
        GyroIO gyroIO,
        SDSModuleIO flModuleIO,
        SDSModuleIO frModuleIO,
        SDSModuleIO blModuleIO,
        SDSModuleIO brModuleIO
    ) {
        this.gyroIO = gyroIO;
        this.gyroIOInputs = new GyroIOInputsAutoLogged();

        toCrossbuck = true;

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
        modulePositions = Arrays.stream(modules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, modulePositions, Constants.getInitialPose());

        aimHubFlag = new AtomicBoolean(false);

        trajVXController = new PIDController(8, 0, 0);
        trajVYController = new PIDController(8, 0, 0);
        trajHeadingController = new PIDController(3, 0, 0);
        trajHeadingController.enableContinuousInput(0, 2 * Math.PI);

        lastMove = Seconds.mutable(Timer.getFPGATimestamp());

        field = new Field2d();
        SmartDashboard.putData("Odometry/Field", field);
    }

    private double adjustAxisInput(
        double controllerInput,
        double deadband,
        double minThreshold,
        double steepness
    ) {
        // see https://www.desmos.com/calculator/wj59z401tq
        
        return
            Math.abs(controllerInput) > deadband ?
                MathUtil.clamp(
                    Math.signum(controllerInput) * (
                        (1 - minThreshold) *
                        Math.pow(
                            (Math.abs(controllerInput) - deadband) / (1 - deadband),
                            steepness
                        ) +
                        minThreshold
                    ),
                    -1,
                    1
                )
            : 0;
    }

    /**
     * Run swerve drive using joystick inputs
     * 
     * @param xInput controller x-axis supplier
     * @param yInput controller y-axis supplier
     * @param omegaInput controller turning supplier
     * @param speedFactorInput analog speed erosion supplier
     * @return
     */
    public Command runDriveInputs(
        DoubleSupplier xInput,
        DoubleSupplier yInput,
        DoubleSupplier omegaInput,
        DoubleSupplier speedFactorInput
    ) {
        return run(() -> {
            double xInputValue = xInput.getAsDouble();
            double yInputValue = yInput.getAsDouble();
            double omegaInputValue = omegaInput.getAsDouble();

            // see https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html#joystick-class
            yInputValue *= -1; // y-axis is inverted on joystick
            omegaInputValue *= -1; // rotation is reversed

            double speedFactor = 1 - (1 - SwerveConstants.kSlowedMult) * speedFactorInput.getAsDouble();

            // apply joysticks to NWU
            double vx = yInputValue;
            double vy = -xInputValue;
            double omega = omegaInputValue;

            double mag = Math.hypot(vx, vy);
            double dir = Math.atan2(vy, vx);

            double deadband = 0.2; // minimum axis input before robot input
            double minThreshold = 0.03; // minimum robot input to overcome resistance
            double steepness = 1.8; // more precision on lower values
            
            mag = adjustAxisInput(mag, deadband, minThreshold, steepness);
            mag *= speedFactor;
            omega = adjustAxisInput(omega, deadband, minThreshold, steepness + 1);
            omega *= speedFactor;

            OrientedChassisSpeeds chassisSpeeds = new OrientedChassisSpeeds(
                SwerveConstants.kMagVelLimit.times(mag * Math.cos(dir)),
                SwerveConstants.kMagVelLimit.times(mag * Math.sin(dir)),
                SwerveConstants.kRotVelLimit.times(omega),
                false, true
            );

            adjustSpeedsForPresetRotation(chassisSpeeds);
            submitChassisSpeeds(chassisSpeeds);
        }).withName("Teleop Drive default");
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

        if (aimHubFlag.get()) {
            speeds.omegaRadiansPerSecond = trajHeadingController.calculate(
                robotPose.getRotation().getRadians(),
                targetHeading.getRadians()
            );
        }
    }

    private void submitChassisSpeeds(
        OrientedChassisSpeeds chassisSpeeds
    ) {
        if (
            chassisSpeeds.vxMetersPerSecond != 0 ||
            chassisSpeeds.vyMetersPerSecond != 0 ||
            chassisSpeeds.omegaRadiansPerSecond != 0
        ) {
            lastMove.mut_replace(Timer.getFPGATimestamp(), Seconds);
        }

        if (
            (toCrossbuck && Seconds.of(Timer.getFPGATimestamp()).minus(lastMove).gt(SwerveConstants.crossbuckDelay))
        ) {
            setModulesToCrossbuckPosition(true);
            return;
        }

        ChassisSpeeds adjustedSpeeds = chassisSpeeds;

        if (chassisSpeeds.fieldCentric) {
            adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds,
                getPose().getRotation().rotateBy(new Rotation2d(Constants.isRed() && !chassisSpeeds.allianceFlipped ? Math.PI : 0))
            );
        }

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

    private void setModulesToCrossbuckPosition(boolean optimize) {
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

    public Command runStopDrive() {
        return runOnce(() -> {
            for (SDSSwerveModule module : modules) {
                module.stopDrive();
            }
        });
    }

    public Command runToggleAimHub(boolean on) {
        return runOnce(() -> {
            aimHubFlag.set(on);
        });
    }

    public Command runSpeedAdjustOnly() {
        return run(() -> {
            var cs = new OrientedChassisSpeeds();
            adjustSpeedsForPresetRotation(cs);
            submitChassisSpeeds(cs);
        });
    }

    @AutoLogOutput(key = "Odometry/Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the robot's odometry to the given pose.
     * Used by ChoreoLib at the start of an autonomous routine.
     *
     * @param pose the field-relative pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
            rawGyroRotation,       // current raw gyro angle (not zeroed)
            modulePositions,       // current module positions
            pose                   // target pose to reset to
        );
    }

    /**
     * Follows a Choreo swerve trajectory sample.
     * Combines feedforward chassis speeds from the sample with
     * PID feedback to correct positional error in real time.
     *
     * @param sample the trajectory sample to follow at this timestep
     */
    public void followTrajectory(SwerveSample sample) {
        Pose2d currentPose = getPose();

        OrientedChassisSpeeds speeds = new OrientedChassisSpeeds(
            sample.vx + trajVXController.calculate(currentPose.getX(), sample.x),
            sample.vy + trajVYController.calculate(currentPose.getY(), sample.y),
            sample.omega + trajHeadingController.calculate(
                currentPose.getRotation().getRadians(),
                sample.heading
            ),
            true, true
        );

        Logger.recordOutput("Swerve/ChassisSpeeds/Auto", speeds);

        adjustSpeedsForPresetRotation(speeds);
        submitChassisSpeeds(speeds);
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp, Matrix<N3,N1> stdDevs) {
        // higher standard deviations means vision measurements are trusted less
        poseEstimator.addVisionMeasurement(visionMeasurement, timestamp, stdDevs);
        getPose();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Swerve/AimHubFlag", aimHubFlag.get());
        Logger.recordOutput("Swerve/CrossbuckEnabled", toCrossbuck);

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

        field.setRobotPose(getPose());

        if (DriverStation.isDisabled()) {
            aimHubFlag.set(false);
        }

        Logger.recordOutput("Swerve/Drive_Command", this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName());
    }
}
