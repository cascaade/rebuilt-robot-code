package frc.robot.subsystems.climb;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.TunableControlConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

public class ClimbConstants {
    public static enum ClimbPose {
        RETRACTED(Radians.of(2 * Math.PI)),
        EXTENDED(Radians.of(0));

        private final Angle setpointAngle;

        ClimbPose(Angle setpointAngle) {
            this.setpointAngle = setpointAngle;
        }

        public Angle getSetpoint() {
            return setpointAngle;
        }
    }

    public static final double CLIMBER_ZERO_VELOCITY_THRESHOLD = 0;
    public static final double CLIMBER_ZERO_VELOCITY_DURATION = 0;
    public static final Voltage CLIMBER_HOMING_VOLTAGE = Volts.of(1);
    public static final double CLIMBER_HOME_RESET_POSITION = 0;

    public static final double CLIMBER_SETPOINT_TOLERANCE = 0;
    public static final double CLIMBER_STOWED_SETPOINT = 0;
    public static final double CLIMBER_READY_SETPOINT = 0;

    public static final int CLIMBER_CAN_ID = 32;

    public static final double CLIMBER_MOTOR_REDUCTION = 3.0 / 1.0;
    public static final double CLIMBER_ENCODER_POSITION_FACTOR = 2 * Math.PI / CLIMBER_MOTOR_REDUCTION;
    public static final double CLIMBER_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 / CLIMBER_MOTOR_REDUCTION;

    public static final TunableControlConstants CLIMBER_CONTROL_CONSTANTS =
        new TunableControlConstants("Climber/Winch")
            .withP(5)
            .withI(0)
            .withD(0);

    public static final SparkMaxConfig CLIMBER_CONFIG = new SparkMaxConfig();

    static {
        CLIMBER_CONFIG
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .voltageCompensation(12)
            .inverted(false);
        CLIMBER_CONFIG.encoder
            .positionConversionFactor(CLIMBER_ENCODER_POSITION_FACTOR)
            .positionConversionFactor(CLIMBER_ENCODER_VELOCITY_FACTOR)
            .uvwAverageDepth(2);
        CLIMBER_CONFIG.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1,1);
        CLIMBER_CONFIG.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        CLIMBER_CONTROL_CONSTANTS.applyTo(CLIMBER_CONFIG);
    }
}