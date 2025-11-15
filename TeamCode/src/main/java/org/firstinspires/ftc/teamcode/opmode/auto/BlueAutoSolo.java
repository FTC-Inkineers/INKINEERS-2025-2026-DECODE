package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Auto Solo")
public class BlueAutoSolo extends MainFarAutonomous {
    @Override
    protected boolean isBlueSide() {
        return true;
    }

    @Override
    protected Variant getVariant() {
        return Variant.SOLO;
    }
}
