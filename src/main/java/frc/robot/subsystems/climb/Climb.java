package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbConstants.ClimbPose;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Volts;

public class Climb extends SubsystemBase {
    public final ClimbIO climbIO;
    public final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    LoggedNetworkNumber volts = new LoggedNetworkNumber("Climber/Volts", 0);

    public Climb(ClimbIO climbIO) {
        this.climbIO = climbIO;
    }

    public Command runExtend() {
        return run(() -> {
            climbIO.setClosedLoop(ClimbPose.EXTENDED.getSetpoint());
        });
    }

    public Command runRetract() {
        return run(() -> {
            climbIO.setClosedLoop(ClimbPose.RETRACTED.getSetpoint());
        });
    }

    public Command runClimb(
        BooleanSupplier posSupplier,
        BooleanSupplier negSupplier
    ) {
        return run(() -> {
             if (posSupplier.getAsBoolean() && negSupplier.getAsBoolean()) {
                 climbIO.setOpenLoop(Volts.of(0));
                 Logger.recordOutput("Climber/SetpointVolts", 0);
             } else if (posSupplier.getAsBoolean()) {
                 climbIO.setOpenLoop(Volts.of(volts.get()));
                 Logger.recordOutput("Climber/SetpointVolts", volts.get());
             } else if (negSupplier.getAsBoolean()) {
                 climbIO.setOpenLoop(Volts.of(-volts.get()));
                 Logger.recordOutput("Climber/SetpointVolts", -volts.get());
             } else {
                 climbIO.setOpenLoop(Volts.of(0));
                 Logger.recordOutput("Climber/SetpointVolts", 0);
             }
        });
    }

    @Override
    public void periodic() {
        climbIO.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
}
