package org.firstinspires.ftc.teamcode.utility;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

/**
 * An action that runs a series of other actions in sequence.
 * It moves to the next action only when the current one is finished.
 */
public class SequentialSail implements Sail {
    private final Queue<Sail> sailQueue;
    private Sail currentSail;

    public SequentialSail(Sail... sails) {
        this.sailQueue = new LinkedList<>(Arrays.asList(sails));
        this.currentSail = null;
    }

    @Override
    public void initialize() {
        // Start the first action in the sequence
        advanceToNextAction();
    }

    @Override
    public void execute() {
        if (currentSail == null) {
            return; // Nothing to do
        }

        // If the current action is finished, move to the next one
        if (currentSail.isFinished()) {
            currentSail.end();
            advanceToNextAction();
        }

        // If there's still a valid action (and it's not finished), execute it
        if (currentSail != null) {
            currentSail.execute();
        }
    }

    private void advanceToNextAction() {
        // Get the next action from the queue
        currentSail = sailQueue.poll();
        if (currentSail != null) {
            // If there is a new action, initialize it
            currentSail.initialize();
        }
    }

    @Override
    public boolean isFinished() {
        // The entire sequence is finished when there are no more actions to run
        return currentSail == null;
    }

    @Override
    public void end() {
        // Ensure the last action's end method is called if it hasn't been already
        if (currentSail != null) {
            currentSail.end();
            currentSail = null;
        }
    }
}
