package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class SDSModuleIOSpark implements SDSModuleIO {
    private final Rotation2d zeroRotation;

    private final SparkMax turnMotor;
    private final SparkMax driveMotor;

    private final RelativeEncoder turnEncoder;
    private final RelativeEncoder driveEncoder;

    private final CANcoder turnCANCoder;
    private final StatusSignal<Angle> turnCANCoderPositionSignal;

    private final SparkClosedLoopController turnController;
    private final SparkClosedLoopController driveController;

    private final boolean driveConnected;
    private final boolean turnConnected;

    private final SlewRateLimiter rateLimiter = new SlewRateLimiter(0.00005);

    private final int index;

    public SDSModuleIOSpark(int index) {
        this.index = index;
        this.zeroRotation = SwerveConstants.zeroRotations[index];

        SparkMaxConfig turnConfig = SwerveConstants.turnConfig;
        SparkMaxConfig driveConfig = SwerveConstants.driveConfig;

        turnMotor = new SparkMax(SwerveConstants.turnCANIDs[index], MotorType.kBrushless);
        driveMotor = new SparkMax(SwerveConstants.driveCANIDs[index], MotorType.kBrushless);

        SwerveConstants.turnCompanion.setMotor(turnMotor);
        SwerveConstants.driveCompanion.setMotor(driveMotor);

        turnMotor.setCANTimeout(0);
        driveMotor.setCANTimeout(0);

        turnEncoder = turnMotor.getEncoder();
        driveEncoder = driveMotor.getEncoder();

        turnController = turnMotor.getClosedLoopController();
        driveController = driveMotor.getClosedLoopController();

        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turnCANCoder = new CANcoder(SwerveConstants.canCoderCANIDs[index]);
        turnCANCoderPositionSignal = turnCANCoder.getAbsolutePosition();
        turnCANCoderPositionSignal.setUpdateFrequency(Constants.odometryFrequency);
        turnCANCoder.optimizeBusUtilization();
        turnEncoder.setPosition(turnCANCoderPositionSignal.getValueAsDouble() * (2 * Math.PI));

        driveConnected = driveMotor.getFirmwareVersion() != 0;
        turnConnected = turnMotor.getFirmwareVersion() != 0;
    }

    @Override
    public void updateInputs(SDSModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(turnCANCoderPositionSignal);

        Angle canCoderPosition = turnCANCoderPositionSignal.getValue();

        turnEncoder.setPosition(canCoderPosition.in(Radians));

        // TODO why dont any electrical signals read? (i.e. appliedvolts & currentamps)
        inputs.turnConnected = turnConnected;
        inputs.turnPosition = new Rotation2d(turnEncoder.getPosition()).minus(zeroRotation);
        inputs.turnVelocity.mut_replace(turnEncoder.getVelocity(), RadiansPerSecond);
        inputs.canCoderPosition.mut_replace(canCoderPosition);
        inputs.turnAppliedVolts.mut_replace(RobotController.getBatteryVoltage() * turnMotor.getAppliedOutput(), Volts);
        inputs.turnCurrentAmps.mut_replace(turnMotor.getOutputCurrent(), Amps);

        inputs.driveConnected = driveConnected;
        inputs.driveDistance.mut_replace(driveEncoder.getPosition(), Meters);
        inputs.driveLinearVelocity.mut_replace(driveEncoder.getVelocity(), MetersPerSecond);
        inputs.driveAppliedVolts.mut_replace(RobotController.getBatteryVoltage() * driveMotor.getAppliedOutput(), Volts);
        inputs.driveCurrentAmps.mut_replace(driveMotor.getOutputCurrent(), Amps);
    }

    // TODO make sure that zero rotation is applied correctly, ensure logic is correct
    @Override
    public void setTurnPosition(Rotation2d position) {
        double setpoint = MathUtil.inputModulus(
            position.plus(zeroRotation).getRadians(),
            SwerveConstants.turnPIDMinInput, SwerveConstants.turnPIDMaxInput
        );
        turnController.setSetpoint(setpoint, ControlType.kPosition);

        Logger.recordOutput("Swerve/AppliedData/Module " + index + "/turnSetpointRadians", setpoint);
        Logger.recordOutput("Swerve/AppliedData/Module " + index + "/atTurnSetpoint", turnController.isAtSetpoint());
    }

    @Override
    public void setDriveVelocity(AngularVelocity velocity) {
        double velocityRadPerSec = velocity.in(RadiansPerSecond);

        if (Math.abs(velocityRadPerSec) < 0.01) velocityRadPerSec = 0;
        double setpointRadPerSec = rateLimiter.calculate(velocityRadPerSec);
        double ffVolts = SwerveConstants.driveKs * Math.signum(velocityRadPerSec) + SwerveConstants.driveKv * velocityRadPerSec;
        driveController.setSetpoint(
            setpointRadPerSec,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts,
            ArbFFUnits.kVoltage
        );

        Logger.recordOutput("Swerve/AppliedData/Module " + index + "/ffVolts", ffVolts);
        Logger.recordOutput("Swerve/AppliedData/Module " + index + "/atDriveSetpoint", driveController.isAtSetpoint());
    }

    @Override
    public void setTurnOpenLoop(Voltage output) {
        turnMotor.setVoltage(output);
    }

    @Override
    public void setDriveOpenLoop(Voltage output) {
        driveMotor.setVoltage(output);
    }
}
