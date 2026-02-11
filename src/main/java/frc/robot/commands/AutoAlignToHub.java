package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Constants.FieldConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoAlignToHub extends Command {
    private final SwerveDrive swerve;

    public AutoAlignToHub(SwerveDrive swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerve.getPose();
        Pose2d hubPose = FieldConstants.getHubCenter();

        Translation2d robotPos = robotPose.getTranslation();
        Translation2d hubPos = hubPose.getTranslation();

        Translation2d robotToHub = hubPos.minus(robotPos);

        Rotation2d targetHeading = robotToHub.getAngle();


    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
