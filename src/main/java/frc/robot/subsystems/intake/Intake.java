package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants.IntakeConstants;
import frc.robot.subsystems.Constants.IntakeConstants.IntakeWristPost;

public class Intake extends SubsystemBase {
    public final RollerIO rollerIO;
    public final WristIO wristIO;

    public Intake(RollerIO rollerIO, WristIO wristIO) {
        this.rollerIO = rollerIO;
        this.wristIO = wristIO;
    }

    public void setPose(IntakeWristPost pose) {
        wristIO.setSetpoint(pose.getSetpoint());
    }

    public void startIntaking() {
        rollerIO.setClosedLoop(IntakeConstants.intakeMaxSpeed);
    }

    public void stopIntaking() {
        rollerIO.setOpenLoop(0);
    }

    @Override
    public void periodic() {
        rollerIO.periodic();
        wristIO.periodic();
    }
}
