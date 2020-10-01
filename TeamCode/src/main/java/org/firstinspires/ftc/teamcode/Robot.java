package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.localization.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.TestDrivetrain;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Vector2d;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class Robot {

    public BNO055IMU imu;
    public ExpansionHubEx controlHub, exHub;
    public ElapsedTime globalTimer = new ElapsedTime();

    public ExpansionHubMotor motor0, motor1, motor2, motor3;

    private Pose2d startPose;
    MathUtil myMath = new MathUtil();

    public boolean initComplete;

    private ThreeWheelLocalizer localizer;
    public TestDrivetrain drive;
    public Vision vision;

    //bulk hub data
    RevBulkData controlData, exData;
    Telemetry telemetry;

    public Robot(HardwareMap hwmap, Pose2d startPose, Telemetry telemetry) {
        //------------------------------------------------------------------
        this.startPose = startPose;
        this.telemetry = telemetry;

        //Initialize all IMU stuff
        imu = hwmap.get(BNO055IMU.class, "IMU");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        motor0 = (ExpansionHubMotor) hwmap.get(DcMotor.class, "motor0");
        motor1 = (ExpansionHubMotor) hwmap.get(DcMotor.class,"motor1");
        motor2 = (ExpansionHubMotor) hwmap.get(DcMotor.class, "motor2");
        motor3 = (ExpansionHubMotor) hwmap.get(DcMotor.class,"motor3");

        controlHub = hwmap.get(ExpansionHubEx.class, "Expansion Hub 2");
        exHub = hwmap.get(ExpansionHubEx.class, "Expansion Hub 3");

        //setup three wheel localizer
        //localizer = new ThreeWheelLocalizer(startPose);

        //adjust the max rpm of "Run Using Encoders" motors
        List<ExpansionHubMotor> RUE_motors = Arrays.asList(motor0,motor1,motor2,motor3);
        for (DcMotor motor: RUE_motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //set zero power behavior for the drive motors
        List<ExpansionHubMotor> driveList = Arrays.asList(motor0,motor1,motor2,motor3);
        for(DcMotor motor: driveList) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //import the motors into the drivetrain subsystem
        drive = new TestDrivetrain
                (motor0, motor1, motor2, motor3, telemetry, imu, exHub, controlHub);

        //Setup vision
        vision = new Vision(hwmap);
    }

    public void initAll() {
        initComplete = false;
        vision.initVision();
        initMotors();
        initServos();
        initComplete = true;
    }

    public void initMotors() {
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initServos() {
        // instantiate servos
    }

    /*
    Get encoder readings in one bulk read
     */
    public void update(Vector2d rotStick, Vector2d transStick) {
        //localizer.update(bulkData);
        drive.updateSwerve(rotStick.getAngle(), transStick.getAngle(), transStick.magnitude());
    }

    public boolean isInitComplete() {
        return initComplete;
    }

    public void resetTimer() {
        globalTimer.reset();
    }

    public double getSeconds() {
        return globalTimer.seconds();
    }

    public double getMillis() { return globalTimer.milliseconds(); }


}