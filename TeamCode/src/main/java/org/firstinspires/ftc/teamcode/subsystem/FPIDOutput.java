package org.firstinspires.ftc.teamcode.subsystem;

public class FPIDOutput {
    public final double f;
    public final double p;
    public final double i;
    public final double d;
    public final double total;

    public FPIDOutput(double f, double p, double i, double d) {
        this.f = f;
        this.p = p;
        this.i = i;
        this.d = d;
        this.total = f + p + i + d;
    }
}
