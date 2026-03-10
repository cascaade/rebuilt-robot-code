package frc.robot.subsystems.swerve;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroIOInputs;
    
    private final SDSSwerveModule[] modules;
    private SwerveModulePosition[] modulePositions;
    
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final PIDController autoXController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController autoYController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController autoHeadingController = new PIDController(7.5, 0.0, 0.0);

    private boolean toX;
    private double lastMove;

    private Rotation2d rawGyroRotation;

    public SwerveDrive(
        GyroIO gyroIO,
        SDSModuleIO flModuleIO,
        SDSModuleIO frModuleIO,
        SDSModuleIO blModuleIO,
        SDSModuleIO brModuleIO
    ) {
        this.gyroIO = gyroIO;
        this.gyroIOInputs = new GyroIOInputsAutoLogged();

        toX = true;

        modules = new SDSSwerveModule[] {
            new SDSSwerveModule("Module 0", flModuleIO),
            new SDSSwerveModule("Module 1", frModuleIO),
            new SDSSwerveModule("Module 2", blModuleIO),
            new SDSSwerveModule("Module 3", brModuleIO)
        };

        kinematics = new SwerveDriveKinematics(
            new Translation2d( SwerveConstants.kWheelDistanceMetersX / 2,  SwerveConstants.kWheelDistanceMetersY / 2),
            new Translation2d( SwerveConstants.kWheelDistanceMetersX / 2, -SwerveConstants.kWheelDistanceMetersY / 2),
            new Translation2d(-SwerveConstants.kWheelDistanceMetersX / 2,  SwerveConstants.kWheelDistanceMetersY / 2),
            new Translation2d(-SwerveConstants.kWheelDistanceMetersX / 2, -SwerveConstants.kWheelDistanceMetersY / 2)
        );

        rawGyroRotation = new Rotation2d();
        modulePositions = Arrays.stream(modules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, modulePositions, SwerveConstants.getInitialPose());
    
        lastMove = Timer.getFPGATimestamp();

        autoHeadingController.enableContinuousInput(0, 2 * Math.PI);
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
            mag *= SwerveConstants.kMagVelLimit * speedFactor;
            omega = adjustAxisInput(omega, deadband, minThreshold, steepness + 1);
            omega *= SwerveConstants.kRotVelLimit * speedFactor;

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(mag * Math.cos(dir), mag * Math.sin(dir), omega);    
            
            runChassisSpeeds(chassisSpeeds);
        });
    }

    public void runChassisSpeeds(
        ChassisSpeeds chassisSpeeds
    ) {
        if (
            chassisSpeeds.vxMetersPerSecond != 0 ||
            chassisSpeeds.vyMetersPerSecond != 0 ||
            chassisSpeeds.omegaRadiansPerSecond != 0
        ) {
            lastMove = Timer.getFPGATimestamp();
        }

        if (toX && Timer.getFPGATimestamp() - lastMove > SwerveConstants.toXDelaySeconds) {
            toXPosition(true);
            return;
        }

        ChassisSpeeds adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            chassisSpeeds,
            getPose().getRotation().rotateBy(new Rotation2d(Constants.isRed() ? Math.PI : 0))
        );

        adjustedSpeeds = ChassisSpeeds.discretize(adjustedSpeeds, LoggedRobot.defaultPeriodSecs);

        SwerveModuleState[] moduleSetpoints = kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleSetpoints, SwerveConstants.kMaxWheelSpeed);

        Logger.recordOutput("Swerve/States/Setpoints", moduleSetpoints);
        Logger.recordOutput("Swerve/ChassisSpeeds/Setpoints", adjustedSpeeds);

        setRawModuleSetpoints(moduleSetpoints, true);
    }

    public void setRawModuleSetpoints(SwerveModuleState[] states, boolean optimize) {
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i], optimize);
        }
    }

    public void toXPosition(boolean optimize) {
        setRawModuleSetpoints(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        }, optimize);
    }

    public Command runToggleToXPosition() {
        return runOnce(() -> {
            toX = !toX;
            Logger.recordOutput("Swerve/toX", toX);
        });
    }

    public Command runZeroGyro() {
        return runOnce(() -> {
            gyroIO.zeroGyro();
        })
        .andThen(new WaitCommand(0.1))
        .andThen(() -> {
            poseEstimator.resetRotation(Constants.isRed() ? Rotation2d.kPi : Rotation2d.kZero);
        });
    }

    public Command runStopDrive() {
        return runOnce(() -> {
            for (SDSSwerveModule module : modules) {
                module.stopDrive();
            }
        });
    }

    @AutoLogOutput(key = "Odometry/Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // ============================================================
    // METHOD 1: resetOdometry(Pose2d)
    // Resets the pose estimator to a known pose.
    // Called by ChoreoLib at the start of every auto routine
    // to align the robot's internal odometry with the trajectory
    // start position.
    // ============================================================

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


    // ============================================================
    // METHOD 2: followTrajectory(SwerveSample)
    // Called every scheduler cycle while a Choreo trajectory
    // is active. Applies the sample's feedforward velocities
    // plus PID correction based on how far off the robot is
    // from the sample's expected pose.
    //
    // This mirrors the pattern from ChoreoLib's documentation
    // and is consistent with your existing runChassisSpeeds()
    // infrastructure.
    // ============================================================

    /**
     * Follows a Choreo swerve trajectory sample.
     * Combines feedforward chassis speeds from the sample with
     * PID feedback to correct positional error in real time.
     *
     * @param sample the trajectory sample to follow at this timestep
     */
    public void followTrajectory(SwerveSample sample) {
        Pose2d currentPose = getPose();

        // Build chassis speeds from the sample's feedforward velocities
        // plus PID correction for x, y, and heading error
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + autoXController.calculate(currentPose.getX(), sample.x),
            sample.vy + autoYController.calculate(currentPose.getY(), sample.y),
            sample.omega + autoHeadingController.calculate(
                currentPose.getRotation().getRadians(),
                sample.heading
            )
        );

        // Log the auto chassis speeds for debugging in AdvantageScope
        Logger.recordOutput("Swerve/ChassisSpeeds/Auto", speeds);

        // Drive field-relative using the existing discretize + kinematics pipeline.
        // NOTE: We call runChassisSpeeds() directly so that the existing
        // ChassisSpeeds.fromFieldRelativeSpeeds() and discretize() logic applies,
        // matching exactly how teleop driving works.
        runChassisSpeeds(speeds);
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp, Matrix<N3,N1> stdDevs) {
        // higher standard deviations means vision measurements are trusted less
        poseEstimator.addVisionMeasurement(visionMeasurement, timestamp, stdDevs);
    }

    /** function that tests module motor controllers by giving them a preset state */
    public Command goofyFunction() {
        return run(() -> {
            for (SDSSwerveModule module : modules) {
                module.setGoofyState();
            }
            // setRawModuleSetpoints(new SwerveModuleState[] {
            //     new SwerveModuleState(0.2, new Rotation2d()),
            //     new SwerveModuleState(0.2, new Rotation2d()),
            //     new SwerveModuleState(0.2, new Rotation2d()),
            //     new SwerveModuleState(0.2, new Rotation2d())
            // }, true);
        });
    }

    public Command runReconfigure() {
        return runOnce(() -> {
            for (SDSSwerveModule module : modules) {
                module.reconfigure();
            }
        });
    }

    @Override
    public void periodic() {
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
    }
}