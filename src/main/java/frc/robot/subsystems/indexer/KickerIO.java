package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface KickerIO {
    @AutoLog
    public class KickerIOInputs {
        public MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

        public MutVoltage appliedVolts = Volts.mutable(0);
        public MutCurrent currentAmps = Amps.mutable(0);
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void syncControlConstants() {}

    public default void setClosedLoop(AngularVelocity velocitySetpoint) {}

    public default void setOpenLoop(Voltage voltage) {}
}
