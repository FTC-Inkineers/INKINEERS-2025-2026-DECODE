package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Auto Push", group = "RedFar")
public class RedAutoPush extends MainFarAutonomous {
    @Override
    protected boolean isBlueSide() {
        return false;
    }

    @Override
    protected Species getVariant() {
        return Species.PUSH;
    }
}
