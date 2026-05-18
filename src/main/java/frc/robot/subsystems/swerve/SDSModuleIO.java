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

import static edu.wpi.first.units.Units.*;

public interface SDSModuleIO {
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

    /** Updates loggable inputs. */
    public default void updateInputs(SDSModuleIOInputs inputs) {}

    public default void syncControlConstants() {}

    /** Set drive velocity setpoint */
    public default void setDriveVelocity(AngularVelocity velocity) {}

    /** Set turn position setpoint */
    public default void setTurnPosition(Rotation2d position) {}

    /** Run turn motor at specified open loop value. */
    public default void setTurnOpenLoop(Voltage output) {}

    /** Run drive motor at specified open loop value. */
    public default void setDriveOpenLoop(Voltage output) {}
}
