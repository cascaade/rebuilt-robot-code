package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog; 

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        // Currents shooter speed in radians/sec
        public double driveVelocityRadPerSec = 0;

        // Motor applied voltage and current
        public double driveAppliedVolts = 0;
        public double driveCurrentAmps = 0;
    }
    
    //Set the shooter motor output [-1.0, 1.0] 
    public default void setDrive(double output) {} 

    //Stops the shooter from moving
    public default void stop() {}; 

    //Read all current inputs into the ShooterIOInputs object 
    public default void updateInputs(ShooterIOInputs inputs) {}; 
}
