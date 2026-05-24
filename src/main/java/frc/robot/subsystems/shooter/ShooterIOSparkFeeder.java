package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.feederConfig;
import static frc.robot.subsystems.shooter.ShooterConstants.feederControlConstants;

public class ShooterIOSparkFeeder implements ShooterIO {

    public final SparkMax feederMotor;
    public final RelativeEncoder feederEncoder;
    public final SparkClosedLoopController feederController;
    public final SparkMaxConfig motorConfig;

    public ShooterIOSparkFeeder(int CANID) {
        feederMotor = new SparkMax(CANID, MotorType.kBrushless);
        feederMotor.setCANTimeout(0);

        feederController = feederMotor.getClosedLoopController();
        feederEncoder = feederMotor.getEncoder();

        motorConfig = ShooterConstants.feederConfig;
        feederMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        syncControlConstants();
    }

    @Override
    public void syncControlConstants() {
        feederControlConstants.applyIfChanged(feederConfig, feederMotor);
    }

    @Override 
    public void setClosedLoop(AngularVelocity velocity) {
        Logger.recordOutput("Shooter/Feeder/Setpoint", velocity);

        // feedforward should already be accounted for
        feederController.setSetpoint(
            velocity.in(RadiansPerSecond),
            ControlType.kVelocity
        );
    }

    @Override 
    public void setOpenLoop(Voltage voltage) {
        feederMotor.setVoltage(voltage);
    }
    
    public void updateInputs(ShooterIOInputs inputs){
        inputs.velocity.mut_replace(feederEncoder.getVelocity(), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(feederMotor.getBusVoltage() * feederMotor.getAppliedOutput(), Volts);
        inputs.supplyCurrentAmps.mut_replace(feederMotor.getOutputCurrent(), Amps);
    }
}
