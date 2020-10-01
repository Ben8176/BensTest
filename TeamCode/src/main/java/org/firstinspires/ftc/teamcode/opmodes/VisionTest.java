package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@TeleOp(name = "Vision Test")
public class VisionTest extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        Vision vision = new Vision(hardwareMap);

       vision.initVision();

        waitForStart();

        vision.stopStreaming();
    }

}
