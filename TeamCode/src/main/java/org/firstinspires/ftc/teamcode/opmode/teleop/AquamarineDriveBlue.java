package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Aquamarine Drive BLUE", group = "TeleOp")
public class AquamarineDriveBlue extends AquamarineDrive {

    @Override
    protected boolean isBlueSide() {
        return true;
    }
}
