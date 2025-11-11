package org.firstinspires.ftc.teamcode.opmode.auto.action;

public class NavigationConsole {
    private Sail currentSail = null;
    private boolean isActionInitialized = false;

    /**
     * Sets a new action to be run.
     * @param sail The action to run.
     */
    public void setSail(Sail sail) {
        this.currentSail = sail;
        // Reset initialization flag for the new action
        this.isActionInitialized = false;
    }

    /**
     * Updates the current action. This should be called in the main loop.
     * It handles the initialize, execute, and end phases of an action's lifecycle.
     */
    public void update() {
        if (currentSail != null) {
            // Initialize the action if it's new
            if (!isActionInitialized) {
                currentSail.initialize();
                isActionInitialized = true;
            }

            // Check if the action is finished
            if (currentSail.isFinished()) {
                currentSail.end(); // Run cleanup
                currentSail = null; // Clear the action
            } else {
                currentSail.execute(); // Continue running the action
            }
        }
    }

    /**
     * Checks if the ActionRunner is currently busy with an action.
     * @return True if an action is running, false otherwise.
     */
    public boolean isBusy() {
        return currentSail != null;
    }
}
