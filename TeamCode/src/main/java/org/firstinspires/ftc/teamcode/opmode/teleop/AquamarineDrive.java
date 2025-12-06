package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystem.AquamarineRobot;

public abstract class AquamarineDrive extends OpMode {

    protected AquamarineRobot robot;
    protected ElapsedTime loopTimer = new ElapsedTime();

    // Configuration is still handled by subclasses
    protected abstract boolean isBlueSide();

    @Override
    public void init() {
        // The robot is now created with the alliance setting
        robot = new AquamarineRobot(hardwareMap, isBlueSide());
        robot.initTeleOp(gamepad1, gamepad2);

        telemetry.addData("Status", "Initialized for " + (isBlueSide() ? "BLUE" : "RED"));
        telemetry.update();
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void init_loop() {
        robot.initLoop();
    }

    @Override
    public void loop() {
        // Track Loop Time
        loopTimer.reset();

        // Delegate the core logic to the robot class
        robot.runTeleOp();
        robot.sendTelemetry(telemetry);

        telemetry.addData("Loop Time (ms)", loopTimer.milliseconds());
        telemetry.update();

        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            // Restore the interrupted status
            Thread.currentThread().interrupt();
        }
    }
}
