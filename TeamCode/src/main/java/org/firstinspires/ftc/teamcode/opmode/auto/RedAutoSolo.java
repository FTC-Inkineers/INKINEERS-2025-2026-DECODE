package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Auto Solo", group = "RedFar")
public class RedAutoSolo extends MainFarAutonomous {
    @Override
    protected boolean isBlueSide() {
        return false;
    }

    @Override
    protected Species getVariant() {
        return Species.SOLO;
    }
}
