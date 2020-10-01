package org.firstinspires.ftc.teamcode.util;

public class MathUtil {

    private final double EPSILON = 1e-6;
    public final static double TAU = 2 * Math.PI;
    public MathUtil() {
    }

    public boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) < EPSILON;
    }

    public double wrapToTau(double angle) {
        return angle % TAU;
    }

    public double wrapTo360(double angle) {
        return angle >= 0 ? angle % 360 : angle + 360;
    }

    public double wrapTo180(double angle) {
        return ((angle + 180) % 360) - 180;
    }

    public double atan360(double y, double x) {
        return wrapTo360(Math.toDegrees(Math.atan2(y, x)));
    }

    public double sinDeg(double angle) {
        return Math.sin(Math.toRadians(angle));
    }

    public double cosDeg(double angle) {
        return Math.cos(Math.toRadians(angle));
    }

    public double magnitude(double x, double y) {
        return Math.sqrt(x*x + y*y);
    }
}