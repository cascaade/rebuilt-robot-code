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

public class ShooterIOSparkFeeder implements ShooterIO {

    public final SparkMax feederMotor;
    public final RelativeEncoder feederEncoder;
    public final SparkClosedLoopController feederController;
    public final SparkMaxConfig motorConfig;

    public final LoggedTunableControlConstants controlConstants = ShooterConstants.feederConstants;

    public ShooterIOSparkFeeder(int CANID) {
        feederMotor = new SparkMax(CANID, MotorType.kBrushless);
        feederMotor.setCANTimeout(0);

        feederController = feederMotor.getClosedLoopController();
        feederEncoder = feederMotor.getEncoder();

        motorConfig = ShooterConstants.feederConfig;
        feederMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controlConstants.addCallback(() -> {
            motorConfig.closedLoop
                .p(controlConstants.kP())
                .d(controlConstants.kD());
            motorConfig.closedLoop.feedForward
                .kV(controlConstants.kV())
                .kS(controlConstants.kS());

            feederMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            System.out.println("Control Constants Updated!");
        });
    }

    @Override 
    public void setVelocityClosedLoop(AngularVelocity velocity) {
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

    @Override 
    public void stop(){
        feederMotor.stopMotor();
    }
    
    public void updateInputs(ShooterIOInputs inputs){
        inputs.velocity.mut_replace(feederEncoder.getVelocity(), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(feederMotor.getBusVoltage() * feederMotor.getAppliedOutput(), Volts);
        inputs.supplyCurrentAmps.mut_replace(feederMotor.getOutputCurrent(), Amps);
    }
}
