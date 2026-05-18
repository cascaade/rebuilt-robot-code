package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

        public MutVoltage appliedVolts = Volts.mutable(0);
        public MutCurrent supplyCurrentAmps = Amps.mutable(0);
    }

    //Read all current inputs into the ShooterIOInputs object
    default void updateInputs(ShooterIOInputs inputs) {
    }

    default void syncControlConstants() {
    }

    // public default void addToOrchestra(Orchestra orchestra, int track) {}

    public default void setVelocityClosedLoop(AngularVelocity velocity) {}

    //Set the shooter motor voltage input
    public default void setOpenLoop(Voltage voltage) {}
}
