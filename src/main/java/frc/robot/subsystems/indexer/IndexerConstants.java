package frc.robot.subsystems.indexer;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.util.TunableControlConstants;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class IndexerConstants {
    public static final int INDEXER_CAN_ID = 29;

    public static final AngularVelocity INDEXER_INTAKE_SPEED = RadiansPerSecond.of(50);
    public static final AngularVelocity INDEXER_OUTTAKE_SPEED = RadiansPerSecond.of(50);

    public static final int INDEXER_CURRENT_LIMIT = 40;

    public static final double INDEXER_MOTOR_REDUCTION = 2.0;
    public static final double INDEXER_ENCODER_POSITION_FACTOR = 2 * Math.PI / INDEXER_MOTOR_REDUCTION;
    public static final double INDEXER_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / INDEXER_MOTOR_REDUCTION / 60;

    public static final TunableControlConstants INDEXER_CONTROL_CONSTANTS =
        new TunableControlConstants("Indexer");

    public static final SparkMaxConfig INDEXER_SPARK_CONFIG = new SparkMaxConfig();

    static {
        INDEXER_SPARK_CONFIG
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12)
            .smartCurrentLimit(INDEXER_CURRENT_LIMIT)
            .closedLoopRampRate(0.02)
            .inverted(true);
        INDEXER_SPARK_CONFIG.encoder
            .positionConversionFactor(INDEXER_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(INDEXER_ENCODER_VELOCITY_FACTOR)
            .uvwAverageDepth(2);
        INDEXER_SPARK_CONFIG.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1);
        INDEXER_SPARK_CONFIG.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        INDEXER_CONTROL_CONSTANTS.applyTo(INDEXER_SPARK_CONFIG);
    }
}
