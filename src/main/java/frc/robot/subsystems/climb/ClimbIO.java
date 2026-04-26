package frc.robot.subsystems.climb;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public class ClimbIOInputs {
        public double positionRadians = 0;
        public double velocityRotPerSec = 0;

        public double appliedVolts = 0;
        public double currentAmps = 0;
    }

    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void setClosedLoop(Angle targetAngle) {}

    public default void setOpenLoop(Voltage voltage) {}
}
