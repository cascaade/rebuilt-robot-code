package frc.robot.subsystems.indexer;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.util.TunableControlConstants;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.NOMINAL_VOLTAGE;

public class IndexerConstants {
    public static class ConveyorConstants {
        public static final int CONVEYOR_CAN_ID = 24;

        public static final AngularVelocity CONVEYOR_FEED_SPEED = RadiansPerSecond.of(50);
        public static final AngularVelocity CONVEYOR_REVERSE_SPEED = RadiansPerSecond.of(50);

        public static final int CONVEYOR_CURRENT_LIMIT = 30;

        public static final double CONVEYOR_MOTOR_REDUCTION = 3.0 / 1.0;
        public static final double CONVEYOR_ENCODER_POSITION_FACTOR = 2 * Math.PI / CONVEYOR_MOTOR_REDUCTION;
        public static final double CONVEYOR_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / CONVEYOR_MOTOR_REDUCTION / 60;

        public static final TunableControlConstants CONVEYOR_CONTROL_CONSTANTS =
            new TunableControlConstants("Indexer/Conveyor")
                .withP(0.00001)
                .withS(0.18)
                .withV(0.00209);

        public static final SparkMaxConfig CONVEYOR_SPARK_CONFIG = new SparkMaxConfig();

        static {
            CONVEYOR_SPARK_CONFIG
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(NOMINAL_VOLTAGE.in(Volts))
                .smartCurrentLimit(CONVEYOR_CURRENT_LIMIT)
                .closedLoopRampRate(0.02)
                .inverted(true);
            CONVEYOR_SPARK_CONFIG.encoder
                .positionConversionFactor(CONVEYOR_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(CONVEYOR_ENCODER_VELOCITY_FACTOR)
                .uvwAverageDepth(2);
            CONVEYOR_SPARK_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-1, 1);
            CONVEYOR_SPARK_CONFIG.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

            CONVEYOR_CONTROL_CONSTANTS.applyTo(CONVEYOR_SPARK_CONFIG);
        }
    }

    public static class KickerConstants {
        public static final int KICKER_CAN_ID = 23;

        public static final AngularVelocity KICKER_FEED_SPEED = RadiansPerSecond.of(50);
        public static final AngularVelocity KICKER_REVERSE_SPEED = RadiansPerSecond.of(50);

        public static final int KICKER_CURRENT_LIMIT = 30;

        public static final double KICKER_MOTOR_REDUCTION = 3.0 / 1.0;
        public static final double KICKER_ENCODER_POSITION_FACTOR = 2 * Math.PI / KICKER_MOTOR_REDUCTION;
        public static final double KICKER_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / KICKER_MOTOR_REDUCTION / 60;

        public static final TunableControlConstants KICKER_CONTROL_CONSTANTS =
            new TunableControlConstants("Indexer/Kicker")
                .withP(0.000001)
                .withS(0.129)
                .withV(0.00203);

        public static final SparkMaxConfig KICKER_SPARK_CONFIG = new SparkMaxConfig();

        static {
            KICKER_SPARK_CONFIG
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(NOMINAL_VOLTAGE.in(Volts))
                .smartCurrentLimit(KICKER_CURRENT_LIMIT)
                .closedLoopRampRate(0.02)
                .inverted(true);
            KICKER_SPARK_CONFIG.encoder
                .positionConversionFactor(KICKER_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(KICKER_ENCODER_VELOCITY_FACTOR)
                .uvwAverageDepth(2);
            KICKER_SPARK_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-1, 1);
            KICKER_SPARK_CONFIG.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

            KICKER_CONTROL_CONSTANTS.applyTo(KICKER_SPARK_CONFIG);
        }
    }
}
