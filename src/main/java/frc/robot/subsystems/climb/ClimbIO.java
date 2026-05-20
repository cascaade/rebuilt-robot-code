package frc.robot.subsystems.climb;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ClimbIO {
    @AutoLog
    public class ClimbIOInputs {
        public MutAngle position = Radians.mutable(0);
        public MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

        public MutVoltage appliedVolts = Volts.mutable(0);
        public MutCurrent currentAmps = Amps.mutable(0);
    }

    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void syncControlConstants() {}

    public default void setClosedLoop(Angle targetAngle) {}

    public default void setOpenLoop(Voltage voltage) {}
}
