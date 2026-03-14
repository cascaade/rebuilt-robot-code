package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.IntakeConstants;

public class WristIOSpark implements WristIO {
    private final SparkMax wristMotor;
    private final SparkClosedLoopController wristController;
    private final RelativeEncoder wristEncoder;
    private final SparkMaxConfig motorConfig;

    private final LoggedNetworkNumber loggedKP = new LoggedNetworkNumber("Intake/Wrist/kP", IntakeConstants.wristP);
    private final LoggedNetworkNumber loggedKD = new LoggedNetworkNumber("Intake/Wrist/kD", IntakeConstants.wristD);
    private final LoggedNetworkNumber loggedKS = new LoggedNetworkNumber("Intake/Wrist/kS", IntakeConstants.wristS);
    private final LoggedNetworkNumber loggedKCos = new LoggedNetworkNumber("Intake/Wrist/kCos", IntakeConstants.wristCos);

    private double lastKp = 0.0;
    private double lastKd = 0.0;
    private double lastKs = 0.0;
    private double lastKcos = 0.0;

    public WristIOSpark() {
        wristMotor = new SparkMax(IntakeConstants.kWristCANID, MotorType.kBrushless);
        wristController = wristMotor.getClosedLoopController();
        wristEncoder = wristMotor.getEncoder();

        motorConfig = IntakeConstants.wristSparkConfig;

        motorConfig.closedLoop
            .pid(IntakeConstants.wristP, 0.0, IntakeConstants.wristD);
        motorConfig.closedLoop.feedForward
            .kCos(IntakeConstants.wristCos).kS(IntakeConstants.wristS);

        wristMotor.configure(IntakeConstants.wristSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        double currentKp = loggedKP.get();
        double currentKd = loggedKD.get();
        double currentKs = loggedKS.get();
        double currentKcos = loggedKCos.get();

        if (currentKp != lastKp || currentKd != lastKd || currentKs != lastKs || currentKcos != lastKcos) {

            motorConfig.closedLoop
                .pid(currentKp, 0.0, currentKd);
            motorConfig.closedLoop.feedForward
                .kCos(currentKcos).kS(currentKs);

            wristMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            lastKp = currentKp;
            lastKd = currentKd;
            lastKs = currentKs;
            lastKcos = currentKcos;

            System.out.println("SparkMax Constants Updated!");
        }
    }

    @Override
    public void resetPosition() {
        wristEncoder.setPosition(0);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.position = new Rotation2d(wristEncoder.getPosition());
        inputs.velocityRadPerSec = wristEncoder.getVelocity();

        inputs.appliedVolts = wristMotor.getBusVoltage() * wristMotor.getAppliedOutput();
        inputs.currentAmps = wristMotor.getOutputCurrent();
    }

    @Override
    public void setSetpoint(double angle) {
        wristController.setSetpoint(angle, ControlType.kPosition);
    }

    @Override
    public void setOpenLoop(double voltage) {
        wristMotor.setVoltage(voltage);
    }
}
