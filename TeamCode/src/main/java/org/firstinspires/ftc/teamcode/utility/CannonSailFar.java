package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public class CannonSailFar extends CannonSail {
    private final double rpm;

    public CannonSailFar(ShooterSubsystem shooter, IntakeSubsystem intake, SequenceMapper.Sequence sequence, int index) {
        super(shooter, intake, sequence, index);

        rpm = shooter.getStationaryRPM_Far();
    }

    @Override
    protected double rpm() {
        return rpm;
    }
}
