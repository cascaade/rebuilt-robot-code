package frc.robot.subsystems.intake;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class IntakeConstants {
    public static class WristConstants {
        public static final int WRIST_CAN_ID = 28;
        public static final int WRIST_CURRENT_LIMIT = 40;
        public static final double WRIST_ZERO_CURRENT_THRESHOLD = 30;
        public static final double WRIST_MOTOR_REDUCTION = 32.0;
        public static final double WRIST_ENCODER_POSITION_FACTOR = 2 * Math.PI / WRIST_MOTOR_REDUCTION;
        public static final double WRIST_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / WRIST_MOTOR_REDUCTION / 60;
        public static final double wristP = 0.2;
        public static final double wristD = 0;
        public static final double wristCos = 0;
        public static final double wristS = 0;
        public static final SparkMaxConfig WRIST_SPARK_CONFIG = new SparkMaxConfig();

        static {
            WRIST_SPARK_CONFIG
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(WRIST_CURRENT_LIMIT)
                .inverted(false);
            WRIST_SPARK_CONFIG.encoder
                .positionConversionFactor(WRIST_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(WRIST_ENCODER_VELOCITY_FACTOR)
                .uvwAverageDepth(2);
            WRIST_SPARK_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(wristP, 0.0, wristD)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-1, 1);
            WRIST_SPARK_CONFIG.closedLoop.feedForward
                .kCos(wristCos).kS(wristS);
            WRIST_SPARK_CONFIG.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        }

        public enum IntakeWristPose {
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
    }

    public static class RollerConstants {
        public static final int ROLLERS_CAN_ID = 29;

        public static final SparkMaxConfig ROLLER_SPARK_CONFIG = new SparkMaxConfig();

        public static final double ROLLERS_MAX_SPEED = 5000; // rad/sec

        public static final int ROLLER_CURRENT_LIMIT = 40;

        public static final double ROLLER_MOTOR_REDUCTION = 2.0;


        public static final double ROLLER_ENCODER_POSITION_FACTOR = 2 * Math.PI / ROLLER_MOTOR_REDUCTION;
        public static final double ROLLER_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / ROLLER_MOTOR_REDUCTION / 60;

        public static final double rollerP = 0.00025;
        public static final double rollerD = 0;
        public static final double rollerS = 0.23;
        public static final double rollerV = 0.0395;

        static {
            ROLLER_SPARK_CONFIG
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12)
                .smartCurrentLimit(ROLLER_CURRENT_LIMIT)
                .closedLoopRampRate(0.02)
                .inverted(true);
            ROLLER_SPARK_CONFIG.encoder
                .positionConversionFactor(ROLLER_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(ROLLER_ENCODER_VELOCITY_FACTOR)
                .uvwAverageDepth(2);
            ROLLER_SPARK_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(rollerP, 0.0, rollerD)
                .outputRange(-1, 1);
            ROLLER_SPARK_CONFIG.closedLoop.feedForward
                .kV(rollerV).kS(rollerS);
            ROLLER_SPARK_CONFIG.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        }
    }
}