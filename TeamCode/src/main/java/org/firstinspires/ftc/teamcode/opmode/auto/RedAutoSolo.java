package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Auto")
public class RedAutoSolo extends MainFarAutonomous {
    @Override
    protected boolean isBlueSide() {
        return false;
    }

    @Override
    protected Variant getVariant() {
        return Variant.SOLO;
    }
}
