package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.indexer.IndexerConstants.ConveyorConstants.CONVEYOR_FEED_SPEED;
import static frc.robot.subsystems.indexer.IndexerConstants.ConveyorConstants.CONVEYOR_REVERSE_SPEED;
import static frc.robot.subsystems.indexer.IndexerConstants.KickerConstants.KICKER_FEED_SPEED;
import static frc.robot.subsystems.indexer.IndexerConstants.KickerConstants.KICKER_REVERSE_SPEED;

public class IndexerSubsystem extends SubsystemBase {
    public enum WantedState {
        IDLE,
        FEED,
        REVERSE
    }

    private enum SystemState {
        IDLING,
        FEEDING,
        REVERSING
        // todo: add jam detection
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    private final ConveyorIO conveyorIO;
    private final KickerIO kickerIO;
    private final ConveyorIOInputsAutoLogged conveyorIOInputs = new ConveyorIOInputsAutoLogged();
    private final KickerIOInputsAutoLogged kickerIOInputs = new KickerIOInputsAutoLogged();

    public IndexerSubsystem(ConveyorIO conveyorIO, KickerIO kickerIO) {
        this.conveyorIO = conveyorIO;
        this.kickerIO = kickerIO;
    }

    private SystemState handleStateTransitions() {
        switch (wantedState) {
            case FEED -> {
                return SystemState.FEEDING;
            }
            case REVERSE -> {
                return SystemState.REVERSING;
            }
            default -> {
                return SystemState.IDLING;
            }
        }
    }

    private void applyStates() {
        switch (systemState) {
            case FEEDING -> {
                conveyorIO.setClosedLoop(CONVEYOR_FEED_SPEED);
                kickerIO.setClosedLoop(KICKER_FEED_SPEED);
            }

            case REVERSING -> {
                conveyorIO.setClosedLoop(CONVEYOR_REVERSE_SPEED);
                kickerIO.setClosedLoop(KICKER_REVERSE_SPEED);
            }

            default -> {
                conveyorIO.setOpenLoop(Volts.of(0));
                kickerIO.setOpenLoop(Volts.of(0));
            }
        }
    }

    public void periodic() {
        this.systemState = handleStateTransitions();
        applyStates();
        this.previousWantedState = wantedState;

        conveyorIO.updateInputs(conveyorIOInputs);
        kickerIO.updateInputs(kickerIOInputs);
        Logger.processInputs("Indexer/Conveyor", conveyorIOInputs);
        Logger.processInputs("Indexer/Kicker", kickerIOInputs);
        conveyorIO.syncControlConstants();
        kickerIO.syncControlConstants();
    }
}
