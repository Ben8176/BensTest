package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Vector2d;

@TeleOp(name = " Driver Control ")
public class DriverControl extends LinearOpMode {

    public Pose2d startPose = new Pose2d(0,0,0);

    @Override
    public void runOpMode() {

        telemetry.addData("Robot Initialized: ", "FALSE");
        telemetry.update();

        Robot robot = new Robot(hardwareMap, startPose, telemetry);
        double prevTime = 0;

        robot.initAll();

        telemetry.addData("Robot Initialized: ", "TRUE");
        telemetry.update();

        //=======================================
        waitForStart();
        //=======================================

        //start the timer when the driver presses play
        robot.resetTimer();

        while (opModeIsActive()) {

            double looptime = robot.getMillis() - prevTime;
            robot.update(new Vector2d(gamepad1.right_stick_x, -gamepad1.right_stick_y),
                    new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y));

            telemetry.addData("time elapsed: ", robot.getMillis());
            telemetry.addData("loop speed (ms)", looptime);
            telemetry.update();


            prevTime = robot.getMillis();
        }
    }
}
