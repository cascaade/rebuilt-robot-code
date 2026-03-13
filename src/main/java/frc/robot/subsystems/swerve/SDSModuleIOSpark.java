package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SDSModuleIOSpark implements SDSModuleIO {
    private final Rotation2d zeroRotation;

    private final SparkMax turnMotor;
    private final SparkMax driveMotor;

    private final RelativeEncoder turnEncoder;
    private final RelativeEncoder driveEncoder;

    private final CANcoder turnCANCoder;

    private final SparkClosedLoopController turnController;
    private final SparkClosedLoopController driveController;

    private double driveKs;
    private double driveKv;

    private final SlewRateLimiter rateLimiter = new SlewRateLimiter(0.00005);

    private final int index;

    public SDSModuleIOSpark(int index) {
        this.index = index;
        this.zeroRotation = SwerveModuleConstants.zeroRotations[index];

        driveKs = SwerveModuleConstants.driveKs;
        driveKv = SwerveModuleConstants.driveKv;

        SparkMaxConfig turnConfig = SwerveModuleConstants.turnConfig;
        SparkMaxConfig driveConfig = SwerveModuleConstants.driveConfig;

        turnMotor = new SparkMax(SwerveConstants.turnCANIDs[index], MotorType.kBrushless);
        driveMotor = new SparkMax(SwerveConstants.driveCANIDs[index], MotorType.kBrushless);

        turnMotor.setCANTimeout(0);
        driveMotor.setCANTimeout(0);

        turnEncoder = turnMotor.getEncoder();
        driveEncoder = driveMotor.getEncoder();

        turnController = turnMotor.getClosedLoopController();
        driveController = driveMotor.getClosedLoopController();

        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // TODO research to figure out if you can save StatusSignal as a variable instead of doing .getAbsolutePosition every time on the CANCoders. what is recommended?
        turnCANCoder = new CANcoder(SwerveConstants.canCoderCANIDs[index]);
        turnEncoder.setPosition(turnCANCoder.getAbsolutePosition().getValueAsDouble() * (2 * Math.PI)); // TODO test to make sure this sets the motor to the correct position
    }
    
    public void updateInputs(SDSModuleIOInputs inputs) {
        turnEncoder.setPosition(turnCANCoder.getAbsolutePosition().getValueAsDouble() * (2 * Math.PI));

        // TODO why dont any electrical signals read? (i.e. appliedvolts & currentamps)
        inputs.turnConnected = turnMotor.getFirmwareVersion() != 0;
        inputs.turnPosition = new Rotation2d(turnEncoder.getPosition()).minus(zeroRotation);
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.canCoderPosition = turnCANCoder.getAbsolutePosition().getValueAsDouble();
        inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();

        inputs.driveConnected = driveMotor.getFirmwareVersion() != 0;
        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.driveVelocityWheelMetersPerSec = inputs.driveVelocityRadPerSec * SwerveModuleConstants.kSwerveWheelDiameter / 2.0;
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
    }

    /** dont spam run */
    public void reconfigure() {
        double driveP = Preferences.getDouble("driveP", 0.0);
        double driveI = Preferences.getDouble("driveI", 0.0);
        double driveD = Preferences.getDouble("driveD", 0.0);
        driveKs = Preferences.getDouble("driveKs", 0);
        driveKv = Preferences.getDouble("driveKv", 0);
        double turnP = Preferences.getDouble("turnP", 0.0);
        double turnI = Preferences.getDouble("turnI", 0.0);
        double turnD = Preferences.getDouble("turnD", 0.0);

        SwerveModuleConstants.turnConfig.closedLoop.pid(turnP, turnI, turnD);
        SwerveModuleConstants.driveConfig.closedLoop.pid(driveP, driveI, driveD);

        turnMotor.configure(SwerveModuleConstants.turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(SwerveModuleConstants.driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    

    // TODO make sure that zero rotation is applied correctly, ensure logic is correct
    public void setTurnPosition(Rotation2d position) {
        double setpoint = MathUtil.inputModulus(
            position.plus(zeroRotation).getRadians(),
            SwerveModuleConstants.turnPIDMinInput, SwerveModuleConstants.turnPIDMaxInput
        );
        turnController.setSetpoint(setpoint, ControlType.kPosition);

        Logger.recordOutput("Swerve/AppliedData/Module " + index + "/turnSetpointRadians", setpoint);
        Logger.recordOutput("Swerve/AppliedData/Module " + index + "/atTurnSetpoint", turnController.isAtSetpoint());
    }

    public void setDriveVelocityRadPerSec(double velocityRadPerSec) {
        if (Math.abs(velocityRadPerSec) < 0.01) velocityRadPerSec = 0;
        double setpointRadPerSec = rateLimiter.calculate(velocityRadPerSec);
        double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
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

    public void setTurnOpenLoop(double output) {
        turnMotor.setVoltage(output);
    }

    public void setDriveOpenLoop(double output) {
        driveMotor.setVoltage(output);
    }
}
