package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public Rotation2d position = new Rotation2d();
        public MutAngularVelocity velocity = RadiansPerSecond.mutable(0);
        
        public MutVoltage appliedVolts = Volts.mutable(0);
        public MutCurrent currentAmps = Amps.mutable(0);
    }

    public default void periodic() {}

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setSetpoint(Rotation2d angleSetpoint) {}

    public default void resetPosition() {}

    public default void setOpenLoop(Voltage voltage) {}
}
