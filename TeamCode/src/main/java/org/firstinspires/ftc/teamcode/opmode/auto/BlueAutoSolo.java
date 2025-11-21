package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Blue Auto Solo", group = "BlueFar")
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
