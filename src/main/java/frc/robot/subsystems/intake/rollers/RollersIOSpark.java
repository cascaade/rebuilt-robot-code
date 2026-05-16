package frc.robot.subsystems.intake.rollers;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.RollerConstants.*;

public class RollersIOSpark implements RollersIO {
    private final SparkMax intakeMotor;
    private final SparkClosedLoopController intakeController;
    private final RelativeEncoder intakeEncoder;
    private final SparkMaxConfig motorConfig;

    private final LoggedNetworkNumber loggedKP = new LoggedNetworkNumber("Intake/Roller/kP", rollerP);
    private final LoggedNetworkNumber loggedKD = new LoggedNetworkNumber("Intake/Roller/kD", rollerD);
    private final LoggedNetworkNumber loggedKS = new LoggedNetworkNumber("Intake/Roller/kS", rollerS);
    private final LoggedNetworkNumber loggedKV = new LoggedNetworkNumber("Intake/Roller/kV", rollerV);

    private double lastKp = 0.0;
    private double lastKd = 0.0;
    private double lastKs = 0.0;
    private double lastKv = 0.0;

    public RollersIOSpark() {
        intakeMotor = new SparkMax(ROLLERS_CAN_ID, MotorType.kBrushless);
        intakeController = intakeMotor.getClosedLoopController();
        intakeEncoder = intakeMotor.getEncoder();

        motorConfig = ROLLER_SPARK_CONFIG;

        motorConfig.closedLoop
            .pid(rollerP, 0.0, rollerD);
        motorConfig.closedLoop.feedForward
            .kV(rollerV).kS(rollerS);

        intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        double currentKp = loggedKP.get();
        double currentKd = loggedKD.get();
        double currentKs = loggedKS.get();
        double currentKv = loggedKV.get();

        if (currentKp != lastKp || currentKd != lastKd || currentKs != lastKs || currentKv != lastKv) {

            motorConfig.closedLoop
                .pid(currentKp, 0.0, currentKd);
            motorConfig.closedLoop.feedForward
                .kV(currentKv).kS(currentKs);

            intakeMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            lastKp = currentKp;
            lastKd = currentKd;
            lastKs = currentKs;
            lastKv = currentKv;

            System.out.println("SparkMax Constants Updated!");
        }
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
        inputs.position.mut_replace(intakeEncoder.getPosition(), Radians);
        inputs.velocity.mut_replace(intakeEncoder.getVelocity(), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput(), Volts);
        inputs.currentAmps.mut_replace(intakeMotor.getOutputCurrent(), Amps);
    }

    @Override
    public void setClosedLoop(AngularVelocity velocity) {
        intakeController.setSetpoint(velocity.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0);

        Logger.recordOutput("Intake/Roller/Setpoint", velocity);
        Logger.recordOutput("Intake/Roller/SetpointUpdate", Timer.getFPGATimestamp());
    }

    @Override
    public void setOpenLoop(Voltage voltage) {
        intakeMotor.setVoltage(voltage);
        Logger.recordOutput("Intake/Roller/Voltage", voltage);
        Logger.recordOutput("Intake/Roller/VoltageUpdate", Timer.getFPGATimestamp());
    }
}
