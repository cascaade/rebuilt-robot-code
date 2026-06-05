package frc.robot.subsystems;

import choreo.trajectory.SwerveSample;
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
        AIMING,
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
        Logger.recordOutput("Superstructure/Booleans/IsDisabled", DriverStation.isDisabled());
        Logger.recordOutput("Superstructure/Booleans/NoJoystick", !DriverStation.isJoystickConnected(0));
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
                if (shooterSubsystem.isAtSpeed() && swerveSubsystem.isAligned()) {
                    return CurrentSuperState.SHOOTING;
                } else {
                    return CurrentSuperState.AIMING;
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
        boolean inNeutralZone = RobotState.getInstance().getRobotFieldPose().getMeasureX().gt(FieldConstants.getHubCenter().getMeasureX());

        ShooterFSM.WantedState shooterState = inNeutralZone ? ShooterFSM.WantedState.PASS : ShooterFSM.WantedState.RESPONSIVE;
        LEDSubsystem.WantedState ledState = LEDSubsystem.WantedState.DISCONNECTED;

        if (Timer.getFPGATimestamp() < 10) {
            ledState = LEDSubsystem.WantedState.BOOT;
        } else if (DriverStation.isDSAttached()) {
            if (DriverStation.isAutonomous())
                ledState = LEDSubsystem.WantedState.AUTONOMOUS;
            else if (DriverStation.isEnabled())
                ledState = LEDSubsystem.WantedState.ENABLED;
            else
                ledState = LEDSubsystem.WantedState.DISABLED;
        }

        swerveSubsystem.requestFollowTrajectory(requestedSwerveSample);

        switch (currentSuperState) {
            case DRIVE -> {
                if (!DriverStation.isAutonomousEnabled()) {
                    swerveSubsystem.setWantedState(SwerveFSM.WantedState.TELEOP);
                } else {
                    swerveSubsystem.setWantedState(SwerveFSM.WantedState.TRAJECTORY);
                    shooterState = ShooterFSM.WantedState.RESPONSIVE;
                    ledState = LEDSubsystem.WantedState.AUTONOMOUS;
                }

                intakeSubsystem.setWantedState(IntakeFSM.WantedState.IDLE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
            }
            case INTAKE -> {
                if (!DriverStation.isAutonomousEnabled()) {
                    swerveSubsystem.setWantedState(SwerveFSM.WantedState.TELEOP);
                } else {
                    swerveSubsystem.setWantedState(SwerveFSM.WantedState.TRAJECTORY);
                }

                intakeSubsystem.setWantedState(IntakeFSM.WantedState.INTAKE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.REVERSE);
            }
            case AIMING -> {
                swerveSubsystem.setWantedState(inNeutralZone ? SwerveFSM.WantedState.AIM_PASS : SwerveFSM.WantedState.AIM_HUB);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.IDLE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.IDLE);
            }
            case SHOOTING -> {
                swerveSubsystem.setWantedState(SwerveFSM.WantedState.CROSS);
                intakeSubsystem.setWantedState(IntakeFSM.WantedState.PULSE);
                indexerSubsystem.setWantedState(IndexerSubsystem.WantedState.FEED);
                ledState = LEDSubsystem.WantedState.BOOT;
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
                ledState = LEDSubsystem.WantedState.DISPLAY_OFF;
            }
            case DISABLED -> {
                requestedSwerveSample = null;

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
