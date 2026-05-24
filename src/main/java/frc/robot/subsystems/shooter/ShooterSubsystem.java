package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.ShooterLUT;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO shooterIOL;
    private final ShooterIO shooterIOM;
    private final ShooterIO shooterIOR;
    private final ShooterIO feederIO;
    private final ShooterIO indexIO;

    private final LoggedNetworkNumber loggedFlywheelRadPerSec = new LoggedNetworkNumber("Shooter/Flywheel/Tuning/Setpoint", ShooterConstants.shooterRunSpeed.in(RadiansPerSecond));
    private final LoggedNetworkNumber loggedFeederRadPerSec = new LoggedNetworkNumber("Shooter/Feeder/Tuning/Setpoint", ShooterConstants.feederRunSpeed.in(RadiansPerSecond));
    private final LoggedNetworkNumber loggedIndexRadPerSec = new LoggedNetworkNumber("Shooter/Index/Tuning/Setpoint", ShooterConstants.indexRunSpeed.in(RadiansPerSecond));

    private final ShooterIOInputsAutoLogged[] shooterInputs = new ShooterIOInputsAutoLogged[5];

    private final MutDistance shooterDistanceAdjust = Meters.mutable(0);
    private boolean runShooterFlag = true;
    private boolean reverseFeeder = false;
    public boolean runIndexFlag = false;
    private boolean maxShooterFlag = false;

    private final Supplier<Pose2d> robotPoseSupplier;

    public ShooterSubsystem(ShooterIO shooterIOL, ShooterIO shooterIOM, ShooterIO shooterIOR, ShooterIO feederIO, ShooterIO indexIO, Supplier<Pose2d> robotPoseSupplier) {
        this.shooterIOL = shooterIOL;
        this.shooterIOM = shooterIOM;
        this.shooterIOR = shooterIOR;
        this.feederIO = feederIO;
        this.indexIO = indexIO;

        this.robotPoseSupplier = robotPoseSupplier;

        for (int i = 0; i < shooterInputs.length; i++) {
            shooterInputs[i] = new ShooterIOInputsAutoLogged();
        }
    }

    private void shootWithDistance(Distance distance) {
        AngularVelocity shooterVelocity = ShooterLUT.getFlywheelSpeedAtDistance(distance.plus(shooterDistanceAdjust));
//        double feederVelocityRadPerSec = shooterVelocityRadPerSec * ShooterConstants.feederMotorMult;

        shooterIOL.setClosedLoop(shooterVelocity);
        shooterIOM.setClosedLoop(shooterVelocity);
        shooterIOR.setClosedLoop(shooterVelocity);

//        if (
//            shooterInputs[0].velocityRadPerSec > shooterVelocityRadPerSec * 0.95 &&
//            shooterInputs[1].velocityRadPerSec > shooterVelocityRadPerSec * 0.95 &&
//            shooterInputs[2].velocityRadPerSec > shooterVelocityRadPerSec * 0.95
//        ) {
//            feederIO.setVelocityClosedLoop(feederVelocityRadPerSec);
//            indexIO.setVelocityClosedLoop(feederVelocityRadPerSec);
//        }
    }

    public Command incrementShooterDistanceAdjust(boolean positive) {
        return runOnce(() -> {
            if (positive) {
                shooterDistanceAdjust.mut_plus(0.2, Meters);
            } else {
                shooterDistanceAdjust.mut_minus(0.2, Meters);
            }
        });
    }

    public Command runStopShooter() {
        return run(() -> {
            shooterIOL.setOpenLoop(Volts.zero());
            shooterIOM.setOpenLoop(Volts.zero());
            shooterIOR.setOpenLoop(Volts.zero());
            feederIO.setOpenLoop(Volts.zero());
            indexIO.setOpenLoop(Volts.zero());
        });
    }

    public Command runShootTheoreticalMaxSpeed() {
        return run(() -> {
            shooterIOL.setClosedLoop(ShooterConstants.shooterRunSpeed);
            shooterIOM.setClosedLoop(ShooterConstants.shooterRunSpeed);
            shooterIOR.setClosedLoop(ShooterConstants.shooterRunSpeed);
            feederIO.setClosedLoop(ShooterConstants.feederRunSpeed);
            indexIO.setClosedLoop(ShooterConstants.feederRunSpeed);
        });
    }

    public Command runAllNominalSpeed() {
        return run(() -> {
            shooterIOL.setOpenLoop(Volts.of(24));
//            shooterIOM.setOpenLoop(12);
//            shooterIOR.setOpenLoop(12);
            feederIO.setOpenLoop(Volts.of(12));
            indexIO.setOpenLoop(Volts.of(12));
        });
    }

    public Command runShooterIdle() {
        return run(() -> {
            shooterIOL.setClosedLoop(ShooterConstants.shooterRunSpeed.times(ShooterConstants.idleMult));
            shooterIOM.setClosedLoop(ShooterConstants.shooterRunSpeed.times(ShooterConstants.idleMult));
            shooterIOR.setClosedLoop(ShooterConstants.shooterRunSpeed.times(ShooterConstants.idleMult));
            // shooterIOL.setVelocityClosedLoop(200);
            // shooterIOM.setVelocityClosedLoop(200);
            // shooterIOR.setVelocityClosedLoop(200);
            feederIO.setOpenLoop(Volts.zero());
            indexIO.setOpenLoop(Volts.zero());
        });
    }

    public Command runShootAtHub(Supplier<Pose2d> poseSupplier) {
        return run(() -> {
            Pose2d robotPose = poseSupplier.get();
            Pose2d hubPose = FieldConstants.getHubCenter();

            Distance hubDistance = Meters.of(robotPose.getTranslation().getDistance(hubPose.getTranslation()));

            shootWithDistance(hubDistance);
        });
    }

    public Command runAllFromNetworkSpeed() {
        return run(() -> {
            shooterIOL.setClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            shooterIOM.setClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            shooterIOR.setClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            feederIO.setClosedLoop(RadiansPerSecond.mutable(loggedFeederRadPerSec.get()));
            indexIO.setClosedLoop(RadiansPerSecond.mutable(loggedIndexRadPerSec.get()));
        });
    }

    public Command runShooterFromNetworkSpeed() {
        return run(() -> {
            shooterIOL.setClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            shooterIOM.setClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            shooterIOR.setClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            // feederIO.setVelocityClosedLoop(loggedFeederRadPerSec.get());
            // indexIO.setVelocityClosedLoop(loggedIndexRadPerSec.get());
        });
    }

    public Command runOtherFromNetworkSpeed() {
        return run(() -> {
            // shooterIOL.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            // shooterIOM.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            // shooterIOR.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            feederIO.setClosedLoop(RadiansPerSecond.mutable(loggedFeederRadPerSec.get()));
            indexIO.setClosedLoop(RadiansPerSecond.mutable(loggedIndexRadPerSec.get()));
        });
    }

    public Command runShootOneShooter() {
        return run(() -> {
            shooterIOL.setClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            feederIO.setClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()).mut_times(ShooterConstants.feederMotorMult));
            indexIO.setClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()).mut_times(ShooterConstants.feederMotorMult));
        });
    }

    public Command toggleRunShooter() {
        return runOnce(() -> {
            runShooterFlag = !runShooterFlag;
        });
    }

    public Command toggleRunIndex(boolean on) {
        return runOnce(() -> {
            runIndexFlag = on;
        });
    }

    public Command runShooterOn() {
        return runOnce(() -> {
            runShooterFlag = true;
        });
    }

    public Command runIndexOn() {
        return runOnce(() -> {
            runIndexFlag = true;
        });
    }

    public Command runToggleReverseFeeder(boolean on) {
        return runOnce(() -> {
            reverseFeeder = on;
        });
    }

    public Command runShooterOff() {
        return runOnce(() -> {
            runShooterFlag = false;
        });
    }

    public Command runIndexOff() {
        return runOnce(() -> {
            runIndexFlag = false;
        });
    }

    public Command runShooterAutonomous(AngularVelocity velocity) {
        return runOnce(() -> {
            shooterIOL.setClosedLoop(velocity);
            shooterIOM.setClosedLoop(velocity);
            shooterIOR.setClosedLoop(velocity);
        });
    }

    public Command runOtherAutonomous(AngularVelocity feederVel, AngularVelocity indexVel) {
        return runOnce(() -> {
            feederIO.setClosedLoop(feederVel);
            indexIO.setClosedLoop(indexVel);
        });
    }

    public Command switchMaxShooterFlag(boolean on) {
        return runOnce(() -> {
            maxShooterFlag = on;
        });
    }

    @Override
    public void periodic() {
        if (runShooterFlag) {
            // shooterIOL.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            // shooterIOM.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            // shooterIOR.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            if(maxShooterFlag) {
                shootWithDistance(Meters.of(178));
            } else {
                Pose2d robotPose = robotPoseSupplier.get();
                Pose2d hubPose = FieldConstants.getHubCenter();

                Distance hubDistance = Meters.of(robotPose.getTranslation().getDistance(hubPose.getTranslation()));

                shootWithDistance(hubDistance);
            }
        } else {
            shooterIOL.setOpenLoop(Volts.zero());
            shooterIOM.setOpenLoop(Volts.zero());
            shooterIOR.setOpenLoop(Volts.zero());
        }
        if (runIndexFlag) {
            double voltageMult = 1
                // (((int) (8 * Timer.getFPGATimestamp())) % 8 == 0)
                // ? 1 : 1
                ;
            feederIO.setClosedLoop(RadiansPerSecond.of(reverseFeeder ? -loggedFeederRadPerSec.get() : loggedFeederRadPerSec.get()));
            indexIO.setClosedLoop(RadiansPerSecond.of(reverseFeeder ? 0 : voltageMult * loggedIndexRadPerSec.get()));
        } else {
            feederIO.setOpenLoop(Volts.zero());
            indexIO.setOpenLoop(Volts.zero());
        }

        Logger.recordOutput("Shooter/ShooterRunning", runShooterFlag);
        Logger.recordOutput("Shooter/IndexRunning", runIndexFlag);
        Logger.recordOutput("Shooter/ShooterDistanceAdjust", shooterDistanceAdjust);

        shooterIOL.syncControlConstants();
        shooterIOM.syncControlConstants();
        shooterIOR.syncControlConstants();
        feederIO.syncControlConstants();
        indexIO.syncControlConstants();

        shooterIOL.updateInputs(shooterInputs[0]);
        shooterIOM.updateInputs(shooterInputs[1]);
        shooterIOR.updateInputs(shooterInputs[2]);
        feederIO.updateInputs(shooterInputs[3]);
        indexIO.updateInputs(shooterInputs[4]);

        Logger.processInputs("Shooter/RightShooter", shooterInputs[0]);
        Logger.processInputs("Shooter/MiddleShooter", shooterInputs[1]);
        Logger.processInputs("Shooter/LeftShooter", shooterInputs[2]);
        Logger.processInputs("Shooter/Feeder", shooterInputs[3]);
        Logger.processInputs("Shooter/Indexer", shooterInputs[4]);

        if (DriverStation.isDisabled()) {
            // runShooterFlag = false;
            runIndexFlag = false;
        }
    }
}
