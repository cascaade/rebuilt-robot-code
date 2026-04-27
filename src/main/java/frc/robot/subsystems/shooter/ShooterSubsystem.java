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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

    // private final Orchestra orchestra;
    // private final Timer disabledTimer = new Timer();

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

        // orchestra = new Orchestra();

        for (int i = 0; i < shooterInputs.length; i++) {
            shooterInputs[i] = new ShooterIOInputsAutoLogged();
        }

        // shooterIOL.addToOrchestra(orchestra, 2);
        // shooterIOM.addToOrchestra(orchestra, 1);
        // shooterIOR.addToOrchestra(orchestra, 0);

        // orchestra.loadMusic("fugue.chrp");
    }

    private void shootWithDistance(Distance distance) {
        AngularVelocity shooterVelocity = ShooterLUT.getFlywheelSpeedAtDistance(distance.plus(shooterDistanceAdjust));
//        double feederVelocityRadPerSec = shooterVelocityRadPerSec * ShooterConstants.feederMotorMult;

        shooterIOL.setVelocityClosedLoop(shooterVelocity);
        shooterIOM.setVelocityClosedLoop(shooterVelocity);
        shooterIOR.setVelocityClosedLoop(shooterVelocity);

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
            shooterIOL.stop();
            shooterIOM.stop();
            shooterIOR.stop();
            feederIO.stop();
            indexIO.stop();
        });
    }

    public Command runShootTheoreticalMaxSpeed() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(ShooterConstants.shooterRunSpeed);
            shooterIOM.setVelocityClosedLoop(ShooterConstants.shooterRunSpeed);
            shooterIOR.setVelocityClosedLoop(ShooterConstants.shooterRunSpeed);
            feederIO.setVelocityClosedLoop(ShooterConstants.feederRunSpeed);
            indexIO.setVelocityClosedLoop(ShooterConstants.feederRunSpeed);
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
            shooterIOL.setVelocityClosedLoop(ShooterConstants.shooterRunSpeed.times(ShooterConstants.idleMult));
            shooterIOM.setVelocityClosedLoop(ShooterConstants.shooterRunSpeed.times(ShooterConstants.idleMult));
            shooterIOR.setVelocityClosedLoop(ShooterConstants.shooterRunSpeed.times(ShooterConstants.idleMult));
            // shooterIOL.setVelocityClosedLoop(200);
            // shooterIOM.setVelocityClosedLoop(200);
            // shooterIOR.setVelocityClosedLoop(200);
            feederIO.stop();
            indexIO.stop();
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
            shooterIOL.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            shooterIOM.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            shooterIOR.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            feederIO.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedFeederRadPerSec.get()));
            indexIO.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedIndexRadPerSec.get()));
        });
    }

    public Command runShooterFromNetworkSpeed() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            shooterIOM.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            shooterIOR.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            // feederIO.setVelocityClosedLoop(loggedFeederRadPerSec.get());
            // indexIO.setVelocityClosedLoop(loggedIndexRadPerSec.get());
        });
    }

    public Command runOtherFromNetworkSpeed() {
        return run(() -> {
            // shooterIOL.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            // shooterIOM.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            // shooterIOR.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            feederIO.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedFeederRadPerSec.get()));
            indexIO.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedIndexRadPerSec.get()));
        });
    }

    public Command runShootOneShooter() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()));
            feederIO.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()).mut_times(ShooterConstants.feederMotorMult));
            indexIO.setVelocityClosedLoop(RadiansPerSecond.mutable(loggedFlywheelRadPerSec.get()).mut_times(ShooterConstants.feederMotorMult));
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
            shooterIOL.setVelocityClosedLoop(velocity);
            shooterIOM.setVelocityClosedLoop(velocity);
            shooterIOR.setVelocityClosedLoop(velocity);
        });
    }

    public Command runOtherAutonomous(AngularVelocity feederVel, AngularVelocity indexVel) {
        return runOnce(() -> {
            feederIO.setVelocityClosedLoop(feederVel);
            indexIO.setVelocityClosedLoop(indexVel);
        });
    }

    public Command switchMaxShooterFlag(boolean on) {
        return runOnce(() -> {
            maxShooterFlag = on;
        });
    }

    @Override
    public void periodic() {
        // if (RobotState.isDisabled()) {
        //     if (!disabledTimer.isRunning()) {
        //         disabledTimer.start();
        //     }

        //     if (disabledTimer.get() >= 30.0) {
        //         if (!orchestra.isPlaying()) {
        //             orchestra.play();
        //         }
        //     }
        // } else {
        //     orchestra.stop();
        //     disabledTimer.stop();
        //     disabledTimer.reset();
        // }

        // Logger.recordOutput("Shooter/OrchestraPlaying", orchestra.isPlaying());
        // Logger.recordOutput("Shooter/DisabledTimer", (int) disabledTimer.get());
        
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
            shooterIOL.stop();
            shooterIOM.stop();
            shooterIOR.stop();
        }
        if (runIndexFlag) {
            double voltageMult = 1
                // (((int) (8 * Timer.getFPGATimestamp())) % 8 == 0)
                // ? 1 : 1
                ;
            feederIO.setVelocityClosedLoop(RadiansPerSecond.of(reverseFeeder ? -loggedFeederRadPerSec.get() : loggedFeederRadPerSec.get()));
            indexIO.setVelocityClosedLoop(RadiansPerSecond.of(reverseFeeder ? 0 : voltageMult * loggedIndexRadPerSec.get()));
        } else {
            feederIO.setOpenLoop(Volts.zero());
            indexIO.setOpenLoop(Volts.zero());
        }

        Logger.recordOutput("Shooter/ShooterRunning", runShooterFlag);
        Logger.recordOutput("Shooter/IndexRunning", runIndexFlag);
        Logger.recordOutput("Shooter/ShooterDistanceAdjust", shooterDistanceAdjust);

        shooterIOL.periodic();
        shooterIOM.periodic();
        shooterIOR.periodic();
        feederIO.periodic();
        indexIO.periodic();

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
