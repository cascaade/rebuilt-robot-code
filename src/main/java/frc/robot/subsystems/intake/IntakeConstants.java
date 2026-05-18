package frc.robot.subsystems.intake;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.TunableControlConstants;

import static edu.wpi.first.units.Units.*;

public class IntakeConstants {
    public static class WristConstants {
        public static final int WRIST_CAN_ID = 28;

        public static final int WRIST_CURRENT_LIMIT = 40;

        public static final AngularVelocity WRIST_ZERO_VELOCITY_THRESHOLD = RadiansPerSecond.of(2);
        public static final Time WRIST_ZERO_VELOCITY_DURATION = Seconds.of(.5);
        public static final Voltage WRIST_HOMING_VOLTAGE = Volts.of(1);
        public static final Angle WRIST_HOME_RESET_POSITION = Radians.of(0);

        public static final Angle WRIST_SETPOINT_TOLERANCE = Radians.of(2);
        public static final Angle WRIST_STOWED_SETPOINT = Radians.of(0);
        public static final Angle WRIST_DEPLOYED_SETPOINT = Radians.of(Math.PI / 2);

        public static final double WRIST_MOTOR_REDUCTION = 32.0;
        public static final double WRIST_ENCODER_POSITION_FACTOR = 2 * Math.PI / WRIST_MOTOR_REDUCTION;
        public static final double WRIST_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / WRIST_MOTOR_REDUCTION / 60;

        public static final Time WRIST_PULSE_STOW_DURATION = Seconds.of(.4);
        public static final Time WRIST_PULSE_DEPLOY_DURATION = Seconds.of(.4);

        public static final TunableControlConstants WRIST_CONTROL_CONSTANTS =
            new TunableControlConstants("Intake/Wrist");

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
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-1, 1);
            WRIST_SPARK_CONFIG.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

            WRIST_CONTROL_CONSTANTS.applyTo(WRIST_SPARK_CONFIG);
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

        public static final AngularVelocity ROLLERS_INTAKE_SPEED = RadiansPerSecond.of(50);
        public static final AngularVelocity ROLLERS_OUTTAKE_SPEED = RadiansPerSecond.of(50); // rad/sec

        public static final int ROLLERS_CURRENT_LIMIT = 40;

        public static final double ROLLERS_MOTOR_REDUCTION = 2.0;
        public static final double ROLLERS_ENCODER_POSITION_FACTOR = 2 * Math.PI / ROLLERS_MOTOR_REDUCTION;
        public static final double ROLLERS_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / ROLLERS_MOTOR_REDUCTION / 60;

        public static final TunableControlConstants ROLLERS_CONTROL_CONSTANTS =
            new TunableControlConstants("Intake/Rollers");

        public static final SparkMaxConfig ROLLERS_SPARK_CONFIG = new SparkMaxConfig();

        static {
            ROLLERS_SPARK_CONFIG
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12)
                .smartCurrentLimit(ROLLERS_CURRENT_LIMIT)
                .closedLoopRampRate(0.02)
                .inverted(true);
            ROLLERS_SPARK_CONFIG.encoder
                .positionConversionFactor(ROLLERS_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(ROLLERS_ENCODER_VELOCITY_FACTOR)
                .uvwAverageDepth(2);
            ROLLERS_SPARK_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1, 1);
            ROLLERS_SPARK_CONFIG.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

            ROLLERS_CONTROL_CONSTANTS.applyTo(ROLLERS_SPARK_CONFIG);
        }
    }
}