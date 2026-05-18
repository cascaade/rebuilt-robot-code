package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.indexConfig;
import static frc.robot.subsystems.shooter.ShooterConstants.indexControlConstants;

public class ShooterIOSparkIndex implements ShooterIO {

    public final SparkMax indexMotor;
    public final RelativeEncoder indexEncoder;
    public final SparkClosedLoopController indexController;

    public ShooterIOSparkIndex(int CANID) {
        indexMotor = new SparkMax(CANID, MotorType.kBrushless);
        indexMotor.setCANTimeout(0);

        indexController = indexMotor.getClosedLoopController();
        indexEncoder = indexMotor.getEncoder();

        syncControlConstants();
    }

    @Override
    public void syncControlConstants() {
        indexControlConstants.applyIfChanged(indexConfig, indexMotor);
    }

    @Override
    public void setVelocityClosedLoop(AngularVelocity velocity) {
        Logger.recordOutput("Shooter/Index/Setpoint", velocity);

        // feedforward should already be accounted for
        indexController.setSetpoint(
            velocity.in(RadiansPerSecond),
            ControlType.kVelocity
        );
    }

    @Override 
    public void setOpenLoop(Voltage voltage) {
        indexMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
        inputs.velocity.mut_replace(indexEncoder.getVelocity(), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(indexMotor.getBusVoltage() * indexMotor.getAppliedOutput(), Volts);
        inputs.supplyCurrentAmps.mut_replace(indexMotor.getOutputCurrent(), Amps);
    }
}
