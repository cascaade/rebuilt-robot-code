package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants.IntakeConstants;
import frc.robot.subsystems.Constants.IntakeConstants.IntakePose;

public class Intake extends SubsystemBase {
    public final IntakeIO intakeIO;
    public final WristIO wristIO;

    public Intake(IntakeIO intakeIO, WristIO wristIO) {
        this.intakeIO = intakeIO;
        this.wristIO = wristIO;
    }

    public void setPose(IntakePose pose) {
        wristIO.setSetpoint(pose.getSetpoint());
    }

    public void startIntaking() {
        intakeIO.setClosedLoop(IntakeConstants.intakeVoltage);
    }

    public void stopIntaking() {
        intakeIO.setOpenLoop(0);
    }
}
