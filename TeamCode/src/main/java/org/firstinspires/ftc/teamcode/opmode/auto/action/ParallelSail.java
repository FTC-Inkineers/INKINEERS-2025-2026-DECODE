package org.firstinspires.ftc.teamcode.opmode.auto.action;

import java.util.Arrays;
import java.util.List;

/**
 * An action that runs multiple actions at the same time.
 * This action is considered finished only when all of its child actions are finished.
 */
public class ParallelSail implements Sail {
    private final List<Sail> sails;

    public ParallelSail(Sail... sails) {
        this.sails = Arrays.asList(sails);
    }

    @Override
    public void initialize() {
        // Initialize all child actions
        for (Sail sail : sails) {
            sail.initialize();
        }
    }

    @Override
    public void execute() {
        // Execute all child actions that are not yet finished
        for (Sail sail : sails) {
            if (!sail.isFinished()) {
                sail.execute();
            }
        }
    }

    @Override
    public boolean isFinished() {
        // The parallel action is finished only if all child actions are finished
        for (Sail sail : sails) {
            if (!sail.isFinished()) {
                return false; // At least one action is still running
            }
        }
        return true; // All actions are finished
    }

    @Override
    public void end() {
        // Call the end method for all child actions
        for (Sail sail : sails) {
            sail.end();
        }
    }
}
