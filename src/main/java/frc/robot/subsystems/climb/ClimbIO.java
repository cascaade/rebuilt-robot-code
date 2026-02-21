package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public class ClimbIOInputs {
        public double positionRotations = 0;
        public double velocityRotPerSec = 0;

        public double appliedVolts = 0;
        public double currentAmps = 0;
    }

    public void updateInputs(ClimbIOInputs inputs);

    public void setSetpoint(double rotations);

    public void setOpenLoop(double voltage);
}
