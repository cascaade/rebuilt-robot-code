package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIOL;
    private final ShooterIO shooterIOM;
    private final ShooterIO shooterIOR;
    private final ShooterIO feederIO;
    private final ShooterIO indexIO;

    private final ShooterIOInputsAutoLogged[] shooterInputs = new ShooterIOInputsAutoLogged[5];

    public Shooter(ShooterIO shooterIOL, ShooterIO shooterIOM, ShooterIO shooterIOR, ShooterIO feederIO, ShooterIO indexIO) {
        this.shooterIOL = shooterIOL;
        this.shooterIOM = shooterIOM;
        this.shooterIOR = shooterIOR;
        this.feederIO = feederIO;
        this.indexIO = indexIO;
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

        shooterIOL.setVelocityClosedLoop(shooterVelocityRadPerSec);
        shooterIOM.setVelocityClosedLoop(shooterVelocityRadPerSec);
        shooterIOR.setVelocityClosedLoop(shooterVelocityRadPerSec);
        feederIO.setVelocityClosedLoop(feederVelocityRadPerSec);
        indexIO.setVelocityClosedLoop(feederVelocityRadPerSec);
    }

    public Command runStopShooter() {
        return runOnce(() -> {
            shooterIOL.stop();
            shooterIOM.stop();
            shooterIOR.stop();
            feederIO.stop();
            indexIO.stop();
        });
    }

    public Command runShootMaxSpeed() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed);
            shooterIOM.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed);
            shooterIOR.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed);
            feederIO.setVelocityClosedLoop(ShooterConstants.feederMotorMaxSpeed);
            indexIO.setVelocityClosedLoop(ShooterConstants.feederMotorMaxSpeed);
        });
    }

    public Command runShooterIdle() {
        return run(() -> {
            shooterIOL.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed * ShooterConstants.idleMult);
            shooterIOM.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed * ShooterConstants.idleMult);
            shooterIOR.setVelocityClosedLoop(ShooterConstants.shooterMotorMaxSpeed * ShooterConstants.idleMult);
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

    @Override
    public void periodic() {
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
