package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystem.AquamarineRobot;

@TeleOp(name = "SUBSYSTEM TESTING", group = "Testing")
public class AquamarineTesting extends OpMode {

    private AquamarineRobot robot;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean isBlueSide = true; // Default to blue

    @Override
    public void init() {
        // Use dashboard for testing
        telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    @Override
    public void init_loop() {
        // This loop runs continuously before START is pressed
        telemetry.addLine("Alliance Selection: Press X for BLUE or B for RED");

        if (gamepad1.x || gamepad2.x) {
            isBlueSide = true;
        } else if (gamepad1.b || gamepad2.b) {
            isBlueSide = false;
        }

        telemetry.addData("Status", "Initialized for " + (isBlueSide ? "BLUE" : "RED"));
        telemetry.update();
    }

    @Override
    public void start() {
        robot = new AquamarineRobot(hardwareMap, isBlueSide);
        robot.initTeleOp(gamepad1, gamepad2);
        robot.start();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        if (robot == null) {
            requestOpModeStop();
            return; // Safety check
        }

        robot.runTeleOp();
        robot.sendTelemetry(telemetry);

        telemetry.addData("Loop Time (ms)", loopTimer.milliseconds());
        telemetry.update();
    }
}
