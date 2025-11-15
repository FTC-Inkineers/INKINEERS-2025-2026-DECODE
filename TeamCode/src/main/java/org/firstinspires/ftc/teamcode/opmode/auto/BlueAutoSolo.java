package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Auto Solo", group = "Far")
public class BlueAutoSolo extends MainFarAutonomous {
    @Override
    protected boolean isBlueSide() {
        return true;
    }

    @Override
    protected Species getVariant() {
        return Species.SOLO;
    }
}
