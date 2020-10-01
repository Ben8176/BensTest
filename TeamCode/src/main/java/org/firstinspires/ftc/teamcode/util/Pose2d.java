package org.firstinspires.ftc.teamcode.util;

import java.util.Vector;

public class Pose2d {
    private final double m_x;
    private final double m_y;
    private final double m_theta;

    public Pose2d() {
        this(0, 0, 0);
    }

    public Pose2d(double x, double y, double theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    public Pose2d(Vector2d pos, double theta) {
        m_x = pos.getX();
        m_y = pos.getY();
        m_theta = theta;
    }

    public Vector2d vec() {
        Vector2d vecComponent = new Vector2d(m_x, m_y);
        return vecComponent;
    }

    public Vector2d headingVec() {
        Vector2d headVec = new Vector2d(Math.cos(m_theta), Math.sin(m_theta));
        return headVec;
    }

    public double getX() {
        return m_x;
    }

    public double getY() {
        return m_y;
    }

    public double getTheta() {
        return m_theta;
    }

}
