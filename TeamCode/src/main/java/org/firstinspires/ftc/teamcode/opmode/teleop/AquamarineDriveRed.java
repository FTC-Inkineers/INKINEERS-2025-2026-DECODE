package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Aquamarine Drive RED", group = "TeleOp")
public class AquamarineDriveRed extends AquamarineDrive {

    @Override
    protected boolean isBlueSide() {
        return false;
    }
}
