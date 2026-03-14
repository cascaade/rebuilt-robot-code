package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.Orchestra;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        // Currents shooter speed in radians/sec
        public double velocityRadPerSec = 0;

        // Motor applied voltage and current
        public double appliedVolts = 0;
        public double supplyCurrentAmps = 0;
    }

    public default void periodic() {}

    public default void addToOrchestra(Orchestra orchestra, int track) {}

    public default void setVelocityClosedLoop(double velocityRadPerSec) {}

    //Set the shooter motor voltage input 
    public default void setOpenLoop(double voltage) {} 

    //Stops the shooter from moving
    public default void stop() {}; 

    //Read all current inputs into the ShooterIOInputs object 
    public default void updateInputs(ShooterIOInputs inputs) {}; 
}
