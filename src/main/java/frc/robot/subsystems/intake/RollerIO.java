package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface RollerIO {
    @AutoLog
    public static class RollerIOInputs {
        public MutAngle position = Radians.mutable(0);
        public MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

        public MutVoltage appliedVolts = Volts.mutable(0);
        public MutCurrent currentAmps = Amps.mutable(0);
    }

    public default void periodic() {}

    public default void updateInputs(RollerIOInputs inputs) {}

    public default void setClosedLoop(AngularVelocity velocity) {}

    public default void setOpenLoop(Voltage voltage) {}
}
