package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.IntakeConstants;

public class RollerIOSpark implements RollerIO {
    private final SparkMax intakeMotor;
    private final SparkClosedLoopController intakeController;
    private final RelativeEncoder intakeEncoder;
    private final SparkMaxConfig motorConfig;

    private final LoggedNetworkNumber loggedKP = new LoggedNetworkNumber("Intake/Roller/kP", IntakeConstants.rollerP);
    private final LoggedNetworkNumber loggedKD = new LoggedNetworkNumber("Intake/Roller/kD", IntakeConstants.rollerD);
    private final LoggedNetworkNumber loggedKS = new LoggedNetworkNumber("Intake/Roller/kS", IntakeConstants.rollerS);
    private final LoggedNetworkNumber loggedKV = new LoggedNetworkNumber("Intake/Roller/kV", IntakeConstants.rollerV);

    private double lastKp = 0.0;
    private double lastKd = 0.0;
    private double lastKs = 0.0;
    private double lastKv = 0.0;

    public RollerIOSpark() {
        intakeMotor = new SparkMax(IntakeConstants.kRollerCANID, MotorType.kBrushless);
        intakeController = intakeMotor.getClosedLoopController();
        intakeEncoder = intakeMotor.getEncoder();

        motorConfig = IntakeConstants.rollerSparkConfig;

        motorConfig.closedLoop
            .pid(IntakeConstants.rollerP, 0.0, IntakeConstants.rollerD);
        motorConfig.closedLoop.feedForward
            .kV(IntakeConstants.rollerV).kS(IntakeConstants.rollerS);

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
    public void updateInputs(RollerIOInputs inputs) {
        inputs.positionRad = intakeEncoder.getPosition();
        inputs.velocityRadPerSec = intakeEncoder.getVelocity();

        inputs.appliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
        inputs.currentAmps = intakeMotor.getOutputCurrent();
    }

    @Override
    public void setClosedLoop(double velocityRadPerSec) {
        intakeController.setSetpoint(velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0);

        Logger.recordOutput("Intake/Roller/Setpoint", velocityRadPerSec);
        Logger.recordOutput("Intake/Roller/SetpointUpdate", Timer.getFPGATimestamp());
    }

    @Override
    public void setOpenLoop(double voltage) {
        intakeMotor.setVoltage(voltage);
        Logger.recordOutput("Intake/Roller/Voltage", voltage);
        Logger.recordOutput("Intake/Roller/VoltageUpdate", Timer.getFPGATimestamp());
    }
    
}
