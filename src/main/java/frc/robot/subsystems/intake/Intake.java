package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    public static enum IntakePose {
        RETRACTED(0),
        EXTENDED(1);

        private final double setpoint;

        IntakePose(double setpoint) {
            this.setpoint = setpoint;
        }

        public double getSetpoint() {
            return setpoint;
        }
    }

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
