package org.firstinspires.ftc.teamcode.opmode.auto.action;

public class ActionRunner {
    private Action currentAction = null;
    private boolean isActionInitialized = false;

    /**
     * Sets a new action to be run.
     * @param action The action to run.
     */
    public void runAction(Action action) {
        this.currentAction = action;
        // Reset initialization flag for the new action
        this.isActionInitialized = false;
    }

    /**
     * Updates the current action. This should be called in the main loop.
     * It handles the initialize, execute, and end phases of an action's lifecycle.
     */
    public void update() {
        if (currentAction != null) {
            // Initialize the action if it's new
            if (!isActionInitialized) {
                currentAction.initialize();
                isActionInitialized = true;
            }

            // Check if the action is finished
            if (currentAction.isFinished()) {
                currentAction.end(); // Run cleanup
                currentAction = null; // Clear the action
            } else {
                currentAction.execute(); // Continue running the action
            }
        }
    }

    /**
     * Checks if the ActionRunner is currently busy with an action.
     * @return True if an action is running, false otherwise.
     */
    public boolean isBusy() {
        return currentAction != null;
    }
}
