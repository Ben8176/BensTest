package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto")
public class Autonomous extends LinearOpMode {


    Pose2d startPose = new Pose2d(0,0,0);
    Robot robot = new Robot(hardwareMap, startPose, telemetry);


    @Override
    public void runOpMode() throws InterruptedException {




        waitForStart();


        while(opModeIsActive()) {

        }
    }
}
