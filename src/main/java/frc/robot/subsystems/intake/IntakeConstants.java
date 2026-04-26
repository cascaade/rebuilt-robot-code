package frc.robot.subsystems.intake;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class IntakeConstants {
    public static final int kWristCANID = 28;
    public static final int kRollerCANID = 29;

    public static final SparkMaxConfig wristSparkConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rollerSparkConfig = new SparkMaxConfig();

    public static final double intakeMaxSpeed = 5000; // rad/sec

    public static final double autoHomeCurrentThreshold = 30;
    public static final int wristCurrentLimit = 40;
    public static final int rollerCurrentLimit = 40;

    public static final double wristMotorReduction = 32.0 / 1.0;
    public static final double rollerMotorReduction = 2.0 / 1.0;

    public static final double wristEncoderPositionFactor = 2 * Math.PI / wristMotorReduction;
    public static final double wristEncoderVelocityFactor = (2 * Math.PI) / wristMotorReduction / 60;
    public static final double rollerEncoderPositionFactor = 2 * Math.PI / rollerMotorReduction;
    public static final double rollerEncoderVelocityFactor = (2 * Math.PI) / rollerMotorReduction / 60;

    public static final double wristP = 0.2;
    public static final double wristD = 0;
    public static final double wristCos = 0;
    public static final double wristS = 0;

    public static final double rollerP = 0.00025;
    public static final double rollerD = 0;
    public static final double rollerS = 0.23;
    public static final double rollerV = 0.0395;

    public static enum IntakeWristPose {
        STOWED(0.1),
        DEPLOYED(1.95);

        private final double setpoint;

        IntakeWristPose(double setpoint) {
            this.setpoint = setpoint;
        }

        public double getSetpoint() {
            return setpoint;
        }
    }

    static {
        wristSparkConfig
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12)
            .smartCurrentLimit(wristCurrentLimit)
            .inverted(false);
        wristSparkConfig.encoder
            .positionConversionFactor(wristEncoderPositionFactor)
            .velocityConversionFactor(wristEncoderVelocityFactor)
            .uvwAverageDepth(2);
        wristSparkConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(wristP, 0.0, wristD)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI)
            .outputRange(-1,1);
        wristSparkConfig.closedLoop.feedForward
            .kCos(wristCos).kS(wristS);
        wristSparkConfig.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        rollerSparkConfig
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12)
            .smartCurrentLimit(rollerCurrentLimit)
            .closedLoopRampRate(0.02)
            .inverted(true);
        rollerSparkConfig.encoder
            .positionConversionFactor(rollerEncoderPositionFactor)
            .velocityConversionFactor(rollerEncoderVelocityFactor)
            .uvwAverageDepth(2);
        rollerSparkConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(rollerP, 0.0, rollerD)
            .outputRange(-1,1);
        rollerSparkConfig.closedLoop.feedForward
            .kV(rollerV).kS(rollerS);
        rollerSparkConfig.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
    }
}