package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private final ShooterIO feederIO;

    public Shooter(ShooterIO shooterIO, ShooterIO feederIO) {
        this.shooterIO = shooterIO;
        this.feederIO = feederIO;
    }

    private double shooterMotorRegressionDistanceToVelocityRadPerSec(double distanceMeters) {
        return distanceMeters; // TODO update regression
    }

    private double feederMotorRegressionDistanceToVelocityRadPerSec(double distanceMeters) {
        return distanceMeters; // TODO update regression
    }

    private void shootWithDistance(double distanceMeters) {
        double shooterVelocityRadPerSec = shooterMotorRegressionDistanceToVelocityRadPerSec(distanceMeters);
        double feederVelocityRadPerSec = feederMotorRegressionDistanceToVelocityRadPerSec(distanceMeters);

        shooterIO.setVelocityClosedLoop(shooterVelocityRadPerSec);
        feederIO.setVelocityClosedLoop(feederVelocityRadPerSec);
    }

    public Command runStopShooter() {
        return runOnce(() -> {
            shooterIO.setOpenLoop(0);
            feederIO.setOpenLoop(0);
        });
    }

    public Command runShootMaxSpeed() {
        return run(() -> {
            shooterIO.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed);
            feederIO.setVelocityClosedLoop(ShooterConstants.feederMotorMaxSpeed);
        });
    }

    public Command runShooterIdle() {
        return run(() -> {
            shooterIO.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed * ShooterConstants.idleMult);
            feederIO.setOpenLoop(0);
        });
    }

    public Command runShootAtHub() {
        return run(() -> {
            // to implement
        });
    }
}
