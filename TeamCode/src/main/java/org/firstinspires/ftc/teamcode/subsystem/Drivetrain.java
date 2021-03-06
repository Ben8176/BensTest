package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.TimerUtil;
import org.firstinspires.ftc.teamcode.util.Vector2d;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.teamcode.util.MathUtil.TAU;

public class Drivetrain {

    MathUtil myMath = new MathUtil();
    TimerUtil myTimer = new TimerUtil();

    Telemetry telemetry;
    ExpansionHubMotor m0, m1, m2, m3;

    //misc. constants
    private final double TRACK_WIDTH = 301; //wheelbasediameter in mm
    private final double WHEEL_DIAMETER = 64; //mm
    private final double MAX_WHEEL_RPM  = 550;
    private double MAX_ROBOT_RPM  = 60;
    private double MAX_MODULE_RPM = 50;
    private final double MAX_MOTOR_RPM  = 1150;
    private final double TICKS_PER_REV = 145.6;

    //Gear ratio constants
    private final double INTERMEDIATE_TO_MOTOR = 16/8;
    private final double MODULE_TO_INTERMEDIATE = 52/13;
    private final double WHEEL_TO_MODULE = 12/48;

    private final double TICKS_PER_MODULE_REV =
            TICKS_PER_REV * INTERMEDIATE_TO_MOTOR * MODULE_TO_INTERMEDIATE;

    // PID constants
    private double kRobot = 2;
    private double kModule = 0.1;
    private double kD = 1;

    int leftSign = 0, rightSign = 0;
    int leftInvert = 1, rightInvert = 1;
    double prevLeftAlpha = 0;
    double prevRightAlpha = 0;
    double prevTime = 0;

    public Drivetrain(ExpansionHubMotor m0, ExpansionHubMotor m1, ExpansionHubMotor m2, ExpansionHubMotor m3, Telemetry telemetry) {
        this.m0 = m0;
        this.m1 = m1;
        this.m2 = m2;
        this.m3 = m3;
        this.telemetry = telemetry;
    }

    public void updateSwerve(Vector2d rotVec, Vector2d transVec, double heading, RevBulkData cdata, RevBulkData edata) {

        int rotSign;
        double robotRPM;

        double currentTime = myTimer.getMillis();

        double m0Encoder =  cdata.getMotorCurrentPosition(0);
        double m1Encoder = -cdata.getMotorCurrentPosition(1);
        double m2Encoder =  edata.getMotorCurrentPosition(0);
        double m3Encoder = -edata.getMotorCurrentPosition(1);

        double rotPower = rotVec.magnitude();
        double rotTheta = Math.toDegrees(rotVec.getAngle());

        double transPower = transVec.magnitude();
        double transTheta = Math.toDegrees(transVec.getAngle());

        //current robot heading
        double rotPhi = heading;
        // difference in desired robot heading and current robot heading
        double rotAlpha = myMath.wrapTo180(rotTheta - rotPhi);

        //
        if (rotAlpha < 0) { rotSign = -1; }
        else if (rotAlpha != 0) { rotSign = 1; }// Needed to avoid divide by 0 error
        else { rotSign = 0; }

        // Left and right module headings as estimated by motor encoders
        double leftModulePhi =
                myMath.wrapTo180(-360 * ((m0Encoder + m1Encoder) / 2) / TICKS_PER_MODULE_REV);
        double rightModulePhi =
                myMath.wrapTo180(-360 * ((m2Encoder + m3Encoder) / 2) / TICKS_PER_MODULE_REV);

        //set the robot RPM to either MAX RPM or some P loop with angle input
        robotRPM = Math.min(kRobot * Math.abs(rotAlpha), MAX_ROBOT_RPM);

        //get module direction in order to turn robot
        double rotLeftTheta = leftModulePhi;
        double rotRightTheta = rightModulePhi;
        if (rotSign != 0) {
            rotLeftTheta = 90 + (90 * rotSign);
            rotRightTheta = 90 - (90 * rotSign);
        }
        else {

        }

        double rotWheelRPM = robotRPM * (TRACK_WIDTH / WHEEL_DIAMETER);

        double transLeftTheta = myMath.wrapTo180(transTheta - rotPhi);
        double transRightTheta = myMath.wrapTo180(transTheta - rotPhi);
        //think of something better to get full powers
        double transWheelRPM = transPower * (MAX_WHEEL_RPM - rotWheelRPM);

        //x y component of left module
        double leftX = rotWheelRPM * myMath.cosDeg(rotLeftTheta) + transWheelRPM * myMath.cosDeg(transLeftTheta);
        double leftY = rotWheelRPM * myMath.sinDeg(rotLeftTheta) + transWheelRPM * myMath.sinDeg(transLeftTheta);
        //x y component of right module
        double rightX = rotWheelRPM * myMath.cosDeg(rotRightTheta) + transWheelRPM * myMath.cosDeg(transRightTheta);
        double rightY = rotWheelRPM * myMath.sinDeg(rotRightTheta) + transWheelRPM * myMath.sinDeg(transRightTheta);

        //get final module headings and RPM from components
        double leftModuleTheta  = myMath.atan360(leftY, leftX);
        double leftWheelRPM     = myMath.magnitude(leftX, leftY);
        double rightModuleTheta = myMath.atan360(rightY, rightX);
        double rightWheelRPM    = myMath.magnitude(rightX, rightY);

        double leftAlpha  = myMath.wrapTo180(leftModuleTheta  - leftModulePhi );
        double rightAlpha = myMath.wrapTo180(rightModuleTheta - rightModulePhi);

        //check which quadrant leftalpha is in
        if (leftAlpha > 90) { //quadrant 2
            leftSign = -1;
            leftInvert = -1;
            leftAlpha = myMath.wrapTo180(leftAlpha + 180);
        }
        else if (leftAlpha < -90) { //quadrant 3
            leftSign = 1;
            leftInvert = -1;
            leftAlpha = myMath.wrapTo180(leftAlpha + 180);
        }
        else if (leftAlpha > 0) { //quadrant 1
            leftSign = 1;
            leftInvert = 1;
            leftAlpha = myMath.wrapTo180(leftAlpha);
        }
        else if (leftAlpha != 0) { //quadrant 4
            leftSign = -1;
            leftInvert = 1;
            leftAlpha = myMath.wrapTo180(leftAlpha);
        }
        else { leftSign = 0; }

        //check which quadrant rightAlpha is in
        if (rightAlpha > 90) { //quadrant 2
            rightSign = -1;
            rightInvert = -1;
            rightAlpha = myMath.wrapTo180(rightAlpha + 180);
        }
        else if (rightAlpha < -90) { //quadrant 3
            rightSign = 1;
            rightInvert = -1;
            rightAlpha = myMath.wrapTo180(rightAlpha + 180);
        }
        else if (rightAlpha > 0) { //quadrant 1
            rightSign = 1;
            rightInvert = 1;
            rightAlpha = myMath.wrapTo180(rightAlpha);
        }
        else if (rightAlpha != 0) { //quadrant 4
            rightSign = -1;
            rightInvert = 1;
            rightAlpha = myMath.wrapTo180(rightAlpha);
        }
        else { rightAlpha = 0; }

        /*
        Derivative control for module turning pid
         */
        double leftModuleD = kD * (leftAlpha - prevLeftAlpha) / (currentTime - prevTime);
        double rightModuleD = kD * (rightAlpha - prevLeftAlpha) / (currentTime - prevTime);

        //get module RPMs based on angle and max module rpm
        double leftRPM  = Math.min(kModule * Math.abs(myMath.wrapTo180(leftAlpha)),  MAX_MODULE_RPM);
        double rightRPM = Math.min(kModule * Math.abs(myMath.wrapTo180(rightAlpha)), MAX_MODULE_RPM);

        //normalize RPMs to be below MAX_MOTOR_RPM
        double wheelModifier =
                1 - (Math.max(leftRPM, rightRPM) * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR) / (MAX_MOTOR_RPM);

        /*
        Get final motor RPMs
         */
        double m0RPM =
                (wheelModifier * leftInvert * leftWheelRPM * WHEEL_TO_MODULE * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR)
                + leftSign * leftRPM * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR;
        //added a negative sign in front of leftInvert for motor1
        double m1RPM =
                (wheelModifier * -leftInvert * leftWheelRPM * WHEEL_TO_MODULE * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR)
                        + leftSign * leftRPM * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR;
        double m2RPM =
                (wheelModifier * rightInvert * rightWheelRPM * WHEEL_TO_MODULE * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR)
                        + rightSign * rightRPM * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR;
        //added a negative sign in front of rightInvert for motor3
        double m3RPM =
                (wheelModifier * -rightInvert * rightWheelRPM * WHEEL_TO_MODULE * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR)
                        + rightSign * rightRPM * MODULE_TO_INTERMEDIATE * INTERMEDIATE_TO_MOTOR;

        //RPM to Radians Per Second conversion
        double RPM_to_RadPerSec = TAU / 60;

        //set motor velocities

//        m0.setVelocity(m0RPM / RPM_to_RadPerSec);
//        m1.setVelocity(m1RPM / RPM_to_RadPerSec);
        m2.setVelocity(m2RPM / RPM_to_RadPerSec);
        m3.setVelocity(m3RPM / RPM_to_RadPerSec);


        prevLeftAlpha = leftAlpha;
        prevRightAlpha = rightAlpha;
        prevTime = currentTime;
    }


}
