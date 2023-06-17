package frc.robot.utils;

public final class PIDGains {
    public double kP;
    public double kI;
    public double kD;
    public PIDGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
