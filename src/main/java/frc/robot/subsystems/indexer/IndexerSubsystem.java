package frc.robot.subsystems.indexer;

import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.indexer.IndexerConstants.INDEXER_INTAKE_SPEED;
import static frc.robot.subsystems.indexer.IndexerConstants.INDEXER_OUTTAKE_SPEED;

public class IndexerSubsystem {
    public enum WantedState {
        IDLE,
        INDEX,
        INDEX_REVERSE
    }

    private enum SystemState {
        IDLING,
        INDEXING,
        INDEXING_REVERSE
        // todo: add jam detection
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged indexerIOInputs = new IndexerIOInputsAutoLogged();

    public IndexerSubsystem(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    private SystemState handleStateTransitions() {
        switch (wantedState) {
            case INDEX -> {
                return SystemState.INDEXING;
            }
            case INDEX_REVERSE -> {
                return SystemState.INDEXING_REVERSE;
            }
            default -> {
                return SystemState.IDLING;
            }
        }
    }

    private void applyStates() {
        switch (systemState) {
            case INDEXING ->
                indexerIO.setClosedLoop(INDEXER_INTAKE_SPEED);

            case INDEXING_REVERSE ->
                indexerIO.setClosedLoop(INDEXER_OUTTAKE_SPEED);

            default ->
                indexerIO.setOpenLoop(Volts.of(0));
        }
    }

    public void periodic() {
        this.systemState = handleStateTransitions();
        applyStates();
        this.previousWantedState = wantedState;

        indexerIO.updateInputs(indexerIOInputs);
        Logger.processInputs("Indexer", indexerIOInputs);
        indexerIO.syncControlConstants();
    }
}
