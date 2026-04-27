
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class SDSSwerveModule {
    private String name;

    private SDSModuleIO io;
    private SDSModuleIOInputsAutoLogged inputs = new SDSModuleIOInputsAutoLogged();
    private SwerveModuleState desiredModuleState;

    public SDSSwerveModule(String name, SDSModuleIO io) {
        Preferences.initBoolean("Swerve Modules/" + name + "enabled", true);
        this.name = name;

        this.io = io;
        desiredModuleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    }

    public void setDesiredState(SwerveModuleState state, boolean optimize) {
        // if(!enabled()) {
        //     stopDrive();
        //     return;
        // }
        if (optimize) {
            state.optimize(inputs.turnPosition);
            state.cosineScale(inputs.turnPosition);
        }

        var angularVelocity = RadiansPerSecond.of(
            state.speedMetersPerSecond / SwerveConstants.kSwerveWheelRadius.in(Meters)
        );

        desiredModuleState = state;
        io.setTurnPosition(state.angle);
        io.setDriveVelocity(angularVelocity);
    }

    public void stopDrive() {
        io.setTurnOpenLoop(Volts.of(0));
        io.setDriveOpenLoop(Volts.of(0));
    }

    /**
     * Gets the current module state based on encoder readings
     *
     * @return
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
            inputs.driveLinearVelocity,
            inputs.turnPosition
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.driveDistance,
            inputs.turnPosition
        );
    }

    public boolean enabled() {
        return Preferences.getBoolean("Swerve Modules/" + name + "enabled", true);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/" + name, inputs);
    }
}
