package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeFSM;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFSM;
import frc.robot.subsystems.swerve.SwerveFSM;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    public enum WantedSuperState {
        DEFAULT,
        AUTO,
        TELEOP,
        INTAKE_TELEOP,
        SHOOT_TELEOP,
        PROTECTED_TELEOP
    }

    public enum CurrentSuperState {
        DEFAULT,
        DISABLED,
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
        if (DriverStation.isDisabled() || !DriverStation.isJoystickConnected(0)) {
            return CurrentSuperState.DISABLED;
        }

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
            case SHOOT_TELEOP -> {
                if (shooterSubsystem.isAtSpeed() && swerveSubsystem.isAligned()) {
                    return CurrentSuperState.SHOOTING_TELEOP;
                } else {
                    return CurrentSuperState.AIMING_TELEOP;
                }
            }
            case PROTECTED_TELEOP -> {
                return CurrentSuperState.PROTECTED_TELEOP;
            }
        }

        return CurrentSuperState.DEFAULT;
    }

    private void applyStates() {
        boolean inNeutralZone = RobotState.getInstance().getRobotFieldPose().getMeasureX().gt(FieldConstants.getHubCenter().getMeasureX());

        ShooterFSM.WantedState shooterState = inNeutralZone ? ShooterFSM.WantedState.PASS : ShooterFSM.WantedState.RESPONSIVE;
        LEDSubsystem.WantedState ledState = LEDSubsystem.WantedState.DISPLAY_OFF;

        if (Timer.getFPGATimestamp() < 10) {
            ledState = LEDSubsystem.WantedState.BOOT;
        } else if (DriverStation.isDSAttached()) {
            if (DriverStation.isAutonomous())
                ledState = LEDSubsystem.WantedState.AUTONOMOUS;
            else if (DriverStation.isEnabled())
                ledState = LEDSubsystem.WantedState.ENABLED;
            else
                ledState = LEDSubsystem.WantedState.DISABLED;
        } else {
            ledState = LEDSubsystem.WantedState.DISCONNECTED;
        }

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
                swerveSubsystem.setWantedState(inNeutralZone ? SwerveFSM.WantedState.AIM_PASS : SwerveFSM.WantedState.AIM_HUB);
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
            case DISABLED -> {
                swerveSubsystem.setWantedState(SwerveFSM.WantedState.STOP);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.IDLE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
                shooterSubsystem.setWantedState(ShooterFSM.WantedState.IDLE);
                ledSubsystem.setWantedState(LEDSubsystem.WantedState.DISABLED);
                return;
            }
        }

        shooterSubsystem.setWantedState(shooterState);
        ledSubsystem.setWantedState(ledState);
    }

    @Override
    public void periodic() {
        this.currentSuperState = handleStateTransitions();
        applyStates();

        Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
        Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    }

    public Command intakeCommand() {
        return this.startEnd(
            () -> setWantedSuperState(WantedSuperState.INTAKE_TELEOP),
            () -> setWantedSuperState(WantedSuperState.TELEOP)
        ).withName("SuperstructureIntake");
    }

    public Command shootCommand() {
        return this.startEnd(
                () -> setWantedSuperState(WantedSuperState.SHOOT_TELEOP),
                () -> setWantedSuperState(WantedSuperState.TELEOP)
        ).withName("SuperstructureShoot");
    }
}
