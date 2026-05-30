package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeFSM;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFSM;
import frc.robot.subsystems.swerve.SwerveFSM;
import lombok.Setter;

public class Superstructure extends SubsystemBase {
    public enum WantedSuperState {
        DEFAULT,
        AUTO,
        TELEOP,
        INTAKE_TELEOP,
        AIM_TELEOP,
        SHOOT_TELEOP,
        PROTECTED_TELEOP
    }

    public enum CurrentSuperState {
        DEFAULT,
        AUTO,
        TELEOP,
        INTAKE_TELEOP,
        AIMING_TELEOP,
        SHOOTING_TELEOP,
        PROTECTED_TELEOP
    }

    @Setter
    private WantedSuperState wantedSuperState = WantedSuperState.DEFAULT;
    private CurrentSuperState currentSuperState = CurrentSuperState.DEFAULT;

    private final SwerveFSM swerveSubsystem;
    private final IntakeFSM intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ShooterFSM shooterSubsystem;
    private final LEDSubsystem ledSubsystem;

    public Superstructure(
        SwerveFSM swerveSubsystem,
        IntakeFSM intakeSubsystem,
        IndexerSubsystem indexerSubsystem,
        ShooterFSM shooterSubsystem,
        LEDSubsystem ledSubsystem
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.ledSubsystem = ledSubsystem;
    }

    private CurrentSuperState handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT -> {
                if (DriverStation.isAutonomousEnabled()) {
                    return CurrentSuperState.AUTO;
                } else if (DriverStation.isTeleopEnabled()) {
                    return CurrentSuperState.TELEOP;
                } else {
                    return CurrentSuperState.DEFAULT;
                }
            }
            case AUTO -> {
                if (!DriverStation.isAutonomousEnabled()) {
                    return CurrentSuperState.DEFAULT;
                }

                return CurrentSuperState.AUTO;
            }
            case TELEOP -> {
                if (!DriverStation.isTeleopEnabled()) {
                    return CurrentSuperState.DEFAULT;
                }

                return CurrentSuperState.TELEOP;
            }
            case INTAKE_TELEOP -> {
                return CurrentSuperState.INTAKE_TELEOP;
            }
            case AIM_TELEOP -> {
                return CurrentSuperState.AIMING_TELEOP;
            }
            case SHOOT_TELEOP -> {
                return CurrentSuperState.SHOOTING_TELEOP;
            }
            case PROTECTED_TELEOP -> {
                return CurrentSuperState.PROTECTED_TELEOP;
            }
        }

        return CurrentSuperState.DEFAULT;
    }

    private void applyStates() {
        boolean usePassPointInsteadOfHub = RobotState.getInstance().getRobotFieldPose().getMeasureX().gt(FieldConstants.getHubCenter().getMeasureX());

        ShooterFSM.WantedState shooterState = usePassPointInsteadOfHub ? ShooterFSM.WantedState.PASS : ShooterFSM.WantedState.RESPONSIVE;
        LEDSubsystem.WantedState ledState = LEDSubsystem.WantedState.DISPLAY_OFF;

        switch (currentSuperState) {
            case DEFAULT -> {
                swerveSubsystem.setWantedState(SwerveFSM.WantedState.STOP);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.IDLE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
            }
            case AUTO -> {
                shooterState = ShooterFSM.WantedState.RESPONSIVE;
                ledState = LEDSubsystem.WantedState.AUTONOMOUS;

                // implementing auto later
            }
            case TELEOP -> {
                swerveSubsystem.setWantedState(SwerveFSM.WantedState.TELEOP);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.IDLE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
            }
            case INTAKE_TELEOP -> {
                swerveSubsystem.setWantedState(SwerveFSM.WantedState.TELEOP);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.INTAKE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.REVERSE);
            }
            case AIMING_TELEOP -> {
                swerveSubsystem.setWantedState(usePassPointInsteadOfHub ? SwerveFSM.WantedState.AIM_PASS : SwerveFSM.WantedState.AIM_HUB);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.IDLE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
            }
            case SHOOTING_TELEOP -> {
                swerveSubsystem.setWantedState(SwerveFSM.WantedState.CROSS);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.PULSE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.FEED);
                ledState = LEDSubsystem.WantedState.BOOT;
            }
            case PROTECTED_TELEOP -> {
                swerveSubsystem.setWantedState(SwerveFSM.WantedState.CROSS);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.IDLE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
            }
        }

        shooterSubsystem.setWantedState(shooterState);
        ledSubsystem.setWantedState(ledState);
    }

    @Override
    public void periodic() {
        this.currentSuperState = handleStateTransitions();
        applyStates();
    }
}
