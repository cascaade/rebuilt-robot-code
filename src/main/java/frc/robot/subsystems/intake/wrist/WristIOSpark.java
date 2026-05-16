package frc.robot.subsystems.intake.wrist;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.WristConstants.*;

public class WristIOSpark implements WristIO {
    private final SparkMax wristMotor;
    private final SparkClosedLoopController wristController;
    private final RelativeEncoder wristEncoder;
    private final SparkMaxConfig motorConfig;

    private final LoggedNetworkNumber loggedKP = new LoggedNetworkNumber("Intake/Wrist/kP", wristP);
    private final LoggedNetworkNumber loggedKD = new LoggedNetworkNumber("Intake/Wrist/kD", wristD);
    private final LoggedNetworkNumber loggedKS = new LoggedNetworkNumber("Intake/Wrist/kS", wristS);
    private final LoggedNetworkNumber loggedKCos = new LoggedNetworkNumber("Intake/Wrist/kCos", wristCos);

    private double lastKp = 0.0;
    private double lastKd = 0.0;
    private double lastKs = 0.0;
    private double lastKcos = 0.0;

    public WristIOSpark() {
        wristMotor = new SparkMax(WRIST_CAN_ID, MotorType.kBrushless);
        wristController = wristMotor.getClosedLoopController();
        wristEncoder = wristMotor.getEncoder();

        motorConfig = WRIST_SPARK_CONFIG;

        motorConfig.closedLoop
            .pid(wristP, 0.0, wristD);
        motorConfig.closedLoop.feedForward
            .kCos(wristCos).kS(wristS);

        wristMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    public void resetPosition(Angle angle) {
        wristEncoder.setPosition(angle.in(Radians));
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.position = new Rotation2d(wristEncoder.getPosition());
        inputs.velocity.mut_replace(wristEncoder.getVelocity(), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(wristMotor.getBusVoltage() * wristMotor.getAppliedOutput(), Volts);
        inputs.currentAmps.mut_replace(wristMotor.getOutputCurrent(), Amps);
    }

    @Override
    public void setSetpoint(Rotation2d angle) {
//        wristController.setSetpoint(angle, ControlType.kPosition);
        Logger.recordOutput("Intake/Wrist/HardwareSetpointAngle", angle);
    }

    @Override
    public void setOpenLoop(Voltage voltage) {
        wristMotor.setVoltage(voltage);
    }
}
