package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ConveyorIO {
    @AutoLog
    public class ConveyorIOInputs {
        public MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

        public MutVoltage appliedVolts = Volts.mutable(0);
        public MutCurrent currentAmps = Amps.mutable(0);
    }

    public default void updateInputs(ConveyorIOInputs inputs) {}

    public default void syncControlConstants() {}

    public default void setClosedLoop(AngularVelocity velocitySetpoint) {}

    public default void setOpenLoop(Voltage voltage) {}
}
