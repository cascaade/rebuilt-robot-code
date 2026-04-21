package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableControlConstants;

import static edu.wpi.first.units.Units.*;

public class ShooterIOSparkIndex implements ShooterIO {

    public final SparkMax indexMotor;
    public final RelativeEncoder indexEncoder;
    public final SparkClosedLoopController indexController;
    public final SparkMaxConfig motorConfig;

    public final LoggedTunableControlConstants controlConstants = ShooterConstants.indexConstants;

    public ShooterIOSparkIndex(int CANID) {
        indexMotor = new SparkMax(CANID, MotorType.kBrushless);
        indexMotor.setCANTimeout(0);

        indexController = indexMotor.getClosedLoopController();
        indexEncoder = indexMotor.getEncoder();

        motorConfig = ShooterConstants.indexConfig;
        indexMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controlConstants.addCallback(() -> {
            motorConfig.closedLoop
                .p(controlConstants.kP())
                .d(controlConstants.kD());
            motorConfig.closedLoop.feedForward
                .kV(controlConstants.kV())
                .kS(controlConstants.kS());

            indexMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            System.out.println("Control Constants Updated!");
        });
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
    public void stop(){
        indexMotor.stopMotor();
    }
    
    public void updateInputs(ShooterIOInputs inputs){
        inputs.velocity.mut_replace(indexEncoder.getVelocity(), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(indexMotor.getBusVoltage() * indexMotor.getAppliedOutput(), Volts);
        inputs.supplyCurrentAmps.mut_replace(indexMotor.getOutputCurrent(), Amps);
    }
}
