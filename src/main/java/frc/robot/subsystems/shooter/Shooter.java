package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.HubShootLUT;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIOL;
    private final ShooterIO shooterIOM;
    private final ShooterIO shooterIOR;
    private final ShooterIO feederIO;
    private final ShooterIO indexIO;

    private final LoggedNetworkNumber loggedFlywheelRadPerSec = new LoggedNetworkNumber("Shooter/Flywheel/Tuning/Setpoint", ShooterConstants.shooterMaxSpeed);
    private final LoggedNetworkNumber loggedFeederRadPerSec = new LoggedNetworkNumber("Shooter/Feeder/Tuning/Setpoint", ShooterConstants.feederMaxSpeed);
    private final LoggedNetworkNumber loggedIndexRadPerSec = new LoggedNetworkNumber("Shooter/Index/Tuning/Setpoint", ShooterConstants.indexMaxSpeed);

    private final ShooterIOInputsAutoLogged[] shooterInputs = new ShooterIOInputsAutoLogged[5];

    private double shooterDistanceAdjust = 0;

    public Shooter(ShooterIO shooterIOL, ShooterIO shooterIOM, ShooterIO shooterIOR, ShooterIO feederIO, ShooterIO indexIO) {
        this.shooterIOL = shooterIOL;
        this.shooterIOM = shooterIOM;
        this.shooterIOR = shooterIOR;
        this.feederIO = feederIO;
        this.indexIO = indexIO;

        for (int i = 0; i < shooterInputs.length; i++) {
            shooterInputs[i] = new ShooterIOInputsAutoLogged();
        }
    }

    private void shootWithDistance(double distanceMeters) {
        double shooterVelocityRadPerSec = HubShootLUT.getFlywheelSpeedAtDistance(distanceMeters + shooterDistanceAdjust);
        double feederVelocityRadPerSec = shooterVelocityRadPerSec * ShooterConstants.feederMotorMult;

        shooterIOL.setVelocityClosedLoop(shooterVelocityRadPerSec);
        shooterIOM.setVelocityClosedLoop(shooterVelocityRadPerSec);
        shooterIOR.setVelocityClosedLoop(shooterVelocityRadPerSec);

        if (
            shooterInputs[0].velocityRadPerSec > shooterVelocityRadPerSec * 0.95 &&
            shooterInputs[1].velocityRadPerSec > shooterVelocityRadPerSec * 0.95 &&
            shooterInputs[2].velocityRadPerSec > shooterVelocityRadPerSec * 0.95
        ) {
            feederIO.setVelocityClosedLoop(feederVelocityRadPerSec);
            indexIO.setVelocityClosedLoop(feederVelocityRadPerSec);
        }
    }

    public Command incrementShooterDistanceAdjust(boolean positive) {
        return runOnce(() -> {
            if (positive) {
                shooterDistanceAdjust += 0.2;
            } else {
                shooterDistanceAdjust -= 0.2;
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
            shooterIOL.setVelocityClosedLoop(ShooterConstants.shooterMaxSpeed);
            shooterIOM.setVelocityClosedLoop(ShooterConstants.shooterMaxSpeed);
            shooterIOR.setVelocityClosedLoop(ShooterConstants.shooterMaxSpeed);
            feederIO.setVelocityClosedLoop(ShooterConstants.feederMaxSpeed);
            indexIO.setVelocityClosedLoop(ShooterConstants.feederMaxSpeed);
        });
    }

    public Command runAllNominalSpeed() {
        return run(() -> {
            shooterIOL.setOpenLoop(24);
//            shooterIOM.setOpenLoop(12);
//            shooterIOR.setOpenLoop(12);
            feederIO.setOpenLoop(12);
            indexIO.setOpenLoop(12);
        });
    }

    public Command runShooterIdle() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(ShooterConstants.shooterMaxSpeed * ShooterConstants.idleMult);
            shooterIOM.setVelocityClosedLoop(ShooterConstants.shooterMaxSpeed * ShooterConstants.idleMult);
            shooterIOR.setVelocityClosedLoop(ShooterConstants.shooterMaxSpeed * ShooterConstants.idleMult);
            feederIO.stop();
            indexIO.stop();
        });
    }

    public Command runShootAtHub(Supplier<Pose2d> poseSupplier) {
        return run(() -> {
            Pose2d robotPose = poseSupplier.get();
            Pose2d hubPose = FieldConstants.getHubCenter();

            double hubDistance = robotPose.getTranslation().getDistance(hubPose.getTranslation());

            shootWithDistance(hubDistance);
        });
    }

    public Command runAllFromNetworkSpeed() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            shooterIOM.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            shooterIOR.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            feederIO.setVelocityClosedLoop(loggedFeederRadPerSec.get());
            indexIO.setVelocityClosedLoop(loggedIndexRadPerSec.get());
        });
    }

    public Command runShootOneShooter() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(loggedFlywheelRadPerSec.get());
            feederIO.setVelocityClosedLoop(loggedFlywheelRadPerSec.get() * ShooterConstants.feederMotorMult);
            indexIO.setVelocityClosedLoop(loggedFlywheelRadPerSec.get() * ShooterConstants.feederMotorMult);
        });
    }

    @Override
    public void periodic() {
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
    }
}
