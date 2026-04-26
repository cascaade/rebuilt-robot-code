package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        // Currents shooter speed in radians/sec
        public MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

        // Motor applied voltage and current
        public MutVoltage appliedVolts = Volts.mutable(0);
        public MutCurrent supplyCurrentAmps = Amps.mutable(0);
    }

    public default void periodic() {}

    // public default void addToOrchestra(Orchestra orchestra, int track) {}

    public default void setVelocityClosedLoop(AngularVelocity velocity) {}

    //Set the shooter motor voltage input 
    public default void setOpenLoop(Voltage voltage) {}

    //Stops the shooter from moving
    public default void stop() {}; 

    //Read all current inputs into the ShooterIOInputs object 
    public default void updateInputs(ShooterIOInputs inputs) {}; 
}
