package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface RollersIO {
    @AutoLog
    public static class RollersIOInputs {
        public MutAngle position = Radians.mutable(0);
        public MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

        public MutVoltage appliedVolts = Volts.mutable(0);
        public MutCurrent currentAmps = Amps.mutable(0);
    }

    public default void periodic() {}

    public default void updateInputs(RollersIOInputs inputs) {}

    public default void setClosedLoop(AngularVelocity velocity) {}

    public default void setOpenLoop(Voltage voltage) {}
}
