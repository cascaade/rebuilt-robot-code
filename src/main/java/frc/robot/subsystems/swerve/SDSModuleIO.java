package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public interface SDSModuleIO {
    /** Set drive velocity setpoint */
    default void setDriveVelocity(AngularVelocity velocity) {
    }

    /** Updates loggable inputs. */
    public default void updateInputs(SDSModuleIOInputs inputs) {}

    /** Set turn position setpoint */
    public default void setTurnPosition(Rotation2d position) {}

    /** Run turn motor at specified open loop value. */
    default void setTurnOpenLoop(Voltage output) {
    }

    /** Run drive motor at specified open loop value. */
    default void setDriveOpenLoop(Voltage output) {
    }

    @AutoLog
    class SDSModuleIOInputs {
        public boolean driveConnected = false;
        public boolean turnConnected = false;

        public Rotation2d turnPosition = new Rotation2d();
        public MutAngularVelocity turnVelocity = RadiansPerSecond.mutable(0);
        public MutAngle canCoderPosition = Radians.mutable(0);

        public MutVoltage turnAppliedVolts = Volts.mutable(0);
        public MutCurrent turnCurrentAmps = Amps.mutable(0);

        public MutDistance driveDistance = Meters.mutable(0);
        public MutLinearVelocity driveLinearVelocity = MetersPerSecond.mutable(0);

        public MutVoltage driveAppliedVolts = Volts.mutable(0);
        public MutCurrent driveCurrentAmps = Amps.mutable(0);
    }
}
