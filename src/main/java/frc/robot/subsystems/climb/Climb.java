package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    public static enum ClimbPose {
        RETRACTED(1),
        EXTENDED(0);

        private final double setpoint;

        ClimbPose(double setpoint) {
            this.setpoint = setpoint;
        }

        public double getSetpoint() {
            return setpoint;
        }
    }

    public ClimbIO climbIO;
    public ClimbIOInputsAutoLogged inputs;

    public Climb(ClimbIO climbIO) {
        this.climbIO = climbIO;
    }

    public Command runExtend() {
        return run(() -> {
            climbIO.setSetpoint(ClimbPose.EXTENDED.getSetpoint());
        });
    }

    public Command runRetract() {
        return run(() -> {
            climbIO.setSetpoint(ClimbPose.RETRACTED.getSetpoint());
        });
    }

    public Command stopClimb() {
        return run(() -> {
            climbIO.setOpenLoop(0);
        });
    }

    @Override
    public void periodic() {
        climbIO.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
}
