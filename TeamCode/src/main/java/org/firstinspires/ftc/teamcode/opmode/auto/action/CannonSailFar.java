package org.firstinspires.ftc.teamcode.opmode.auto.action;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;

public class CannonSailFar extends CannonSail {
    private final double rpm;

    public CannonSailFar(ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem.ObeliskMotif motif, int index) {
        super(shooter, intake, motif, index);

        rpm = shooter.getStationaryRPM_Far();
    }

    @Override
    protected double rpm() {
        return rpm;
    }
}
