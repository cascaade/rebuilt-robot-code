package frc.robot.subsystems.climb;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Radians;

public class ClimbConstants {
    public static enum ClimbPose {
        RETRACTED(Radians.of(2 * Math.PI)),
        EXTENDED(Radians.of(0));

        // radians
        private final Angle setpointAngle;

        ClimbPose(Angle setpointAngle) {
            this.setpointAngle = setpointAngle;
        }

        public Angle getSetpoint() {
            return setpointAngle;
        }
    }

    public static final int climberMotorCANID = 32;

    public static final double climbMotorReduction = 3.0 / 1.0;
    public static final double climbEncoderPositionFactor = 2 * Math.PI / climbMotorReduction;
    public static final double climbEncoderVelocityFactor = (2 * Math.PI) / 60.0 / climbMotorReduction;

    public static final double kP = 5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;

    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
        climberConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .voltageCompensation(12)
            .inverted(false);
        climberConfig.encoder
            .positionConversionFactor(climbEncoderPositionFactor)
            .positionConversionFactor(climbEncoderVelocityFactor)
            .uvwAverageDepth(2);
        climberConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .outputRange(-1,1);
        climberConfig.closedLoop.feedForward
            .kS(kS).kV(kV);
        climberConfig.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
    }
}