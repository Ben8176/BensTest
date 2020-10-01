package org.firstinspires.ftc.teamcode.util;


public class Vector2d {
    private final double m_x;
    private final double m_y;

    public Vector2d() {
        this(0,0);
    }

    public Vector2d(double x, double y) {
        m_x = x;
        m_y = y;
    }

    public Vector2d(Vector2d other) {
        m_x = other.getX();
        m_y = other.getY();
    }

    public Vector2d rotateBy(double angle) {
        angle = Math.toRadians(angle);
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        double x = m_x * cosA - m_y * sinA;
        double y = m_x * sinA - m_y * cosA;
        return new Vector2d(x,y);
    }

    public double getAngle() {
        return Math.atan2(m_y, m_x);
    }

    public double magnitude() {
        return Math.hypot(m_x, m_y);
    }

    public double getX() {
        return m_x;
    }
    public double getY() {
        return m_y;
    }

}
