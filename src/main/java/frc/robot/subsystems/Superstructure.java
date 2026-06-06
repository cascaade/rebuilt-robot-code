package frc.robot.subsystems;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        DISABLED,
        HOME,
        DRIVE,
        INTAKE,
        SHOOT,
        PROTECTED
    }

    public enum CurrentSuperState {
        DISABLED,
        HOMING,
        DRIVE,
        INTAKE,
        AIMING_HUB,
        SHOOTING,
        PROTECTED
    }

    @Setter
    private WantedSuperState wantedSuperState = WantedSuperState.DISABLED;
    private WantedSuperState previousWantedSuperState = WantedSuperState.DISABLED;
    private CurrentSuperState currentSuperState = CurrentSuperState.DISABLED;

    private boolean homeCompleted = false;
    private SwerveSample requestedSwerveSample = null;

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

        if (!homeCompleted) {
            this.wantedSuperState = WantedSuperState.HOME;
        }

        switch (wantedSuperState) {
            case DRIVE -> {
                return CurrentSuperState.DRIVE;
            }
            case INTAKE -> {
                return CurrentSuperState.INTAKE;
            }
            case SHOOT -> {
                if (shooterSubsystem.isAtSpeed() && swerveSubsystem.isAligned() || RobotState.getInstance().isOutsideAllianceZone()) {
                    return CurrentSuperState.SHOOTING;
                } else {
                    return CurrentSuperState.AIMING_HUB;
                }
            }
            case PROTECTED -> {
                return CurrentSuperState.PROTECTED;
            }
            case HOME -> {
                if (previousWantedSuperState != WantedSuperState.HOME) {
                    homeCompleted = false;
                }

                if (!hasHomeCompleted()) {
                    return CurrentSuperState.HOMING;
                }
                homeCompleted = true;
                setWantedSuperState(WantedSuperState.DRIVE);
                return CurrentSuperState.DRIVE;
            }
        }

        return CurrentSuperState.DRIVE;
    }

    private void applyStates() {
        ShooterFSM.WantedState shooterState = RobotState.getInstance().isOutsideAllianceZone() ? ShooterFSM.WantedState.PASS : ShooterFSM.WantedState.RESPONSIVE;
        LEDSubsystem.WantedState ledState = LEDSubsystem.WantedState.DISPLAY_OFF;

        swerveSubsystem.requestFollowTrajectory(requestedSwerveSample);

        switch (currentSuperState) {
            case DRIVE -> {
                if (DriverStation.isAutonomousEnabled()) {
                    swerveSubsystem.setWantedState(SwerveFSM.WantedState.TRAJECTORY);
                    shooterState = ShooterFSM.WantedState.RESPONSIVE;
                    ledState = LEDSubsystem.WantedState.AUTONOMOUS;
                } else {
                    swerveSubsystem.setWantedState(SwerveFSM.WantedState.TELEOP);
                    ledState = LEDSubsystem.WantedState.TELEOP;
                }

                intakeSubsystem.setWantedState(IntakeFSM.WantedState.IDLE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
            }
            case INTAKE -> {
                if (DriverStation.isAutonomousEnabled()) {
                    swerveSubsystem.setWantedState(SwerveFSM.WantedState.TRAJECTORY);
                } else {
                    swerveSubsystem.setWantedState(SwerveFSM.WantedState.TELEOP);
                }

                intakeSubsystem.setWantedState(IntakeFSM.WantedState.INTAKE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.REVERSE);
                ledState = LEDSubsystem.WantedState.INTAKING;
            }
            case AIMING_HUB -> {
                swerveSubsystem.setWantedState(SwerveFSM.WantedState.AIM_HUB);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.IDLE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
                ledState = LEDSubsystem.WantedState.AIMING_HUB;
            }
            case SHOOTING -> {
                if (RobotState.getInstance().isOutsideAllianceZone()) {
                    swerveSubsystem.setWantedState(SwerveFSM.WantedState.AIM_PASS);
                    intakeSubsystem.setWantedState(IntakeFSM.WantedState.INTAKE);
                    ledState = LEDSubsystem.WantedState.PASSING;
                } else {
                    swerveSubsystem.setWantedState(SwerveFSM.WantedState.CROSS);
                    intakeSubsystem.setWantedState(IntakeFSM.WantedState.PULSE);
                    ledState = LEDSubsystem.WantedState.SHOOT;
                }

                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.FEED);
            }
            case PROTECTED -> {
                swerveSubsystem.setWantedState(SwerveFSM.WantedState.CROSS);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.IDLE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
            }
            case HOMING -> {
                swerveSubsystem.setWantedState(SwerveFSM.WantedState.STOP);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.HOME);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
                shooterState = ShooterFSM.WantedState.IDLE;
                ledState = LEDSubsystem.WantedState.HOMING;
            }
            case DISABLED -> {
                requestedSwerveSample = null;

                if (Timer.getFPGATimestamp() < 15) {
                    ledState = LEDSubsystem.WantedState.BOOT;
                } else if (DriverStation.isDSAttached()) {
                    ledState = LEDSubsystem.WantedState.DISABLED;
                } else {
                    ledState = LEDSubsystem.WantedState.DISCONNECTED;
                }

                swerveSubsystem.setWantedState(SwerveFSM.WantedState.STOP);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.IDLE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
                shooterSubsystem.setWantedState(ShooterFSM.WantedState.IDLE);
                ledSubsystem.setWantedState(ledState);
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
        this.previousWantedSuperState = wantedSuperState;

        Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
        Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    }

    public boolean hasHomeCompleted() {
        return intakeSubsystem.hasHomeCompleted();
    }

    public Command intakeCommand() {
        return this.startEnd(
            () -> setWantedSuperState(WantedSuperState.INTAKE),
            () -> setWantedSuperState(WantedSuperState.DRIVE)
        ).withName("SuperstructureIntake");
    }

    public Command shootCommand() {
        return this.startEnd(
                () -> setWantedSuperState(WantedSuperState.SHOOT),
                () -> setWantedSuperState(WantedSuperState.DRIVE)
        ).withName("SuperstructureShoot");
    }

    public Command disableCommand() {
        return runOnce(
            () -> setWantedSuperState(WantedSuperState.DISABLED)
        );
    }

    public void requestFollowTrajectory(SwerveSample sample) {
        requestedSwerveSample = sample;
    }
}
