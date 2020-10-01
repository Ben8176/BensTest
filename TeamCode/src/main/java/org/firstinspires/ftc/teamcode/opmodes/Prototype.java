package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "Prototpe")
public class Prototype extends LinearOpMode {

    public DcMotorEx flywheel, intake;
    public CRServo index;

//    PrototypeRobot proto = new PrototypeRobot(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {

        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        index = hardwareMap.get(CRServo.class, "index");

        waitForStart();

        while (opModeIsActive()) {
            intake.setPower(gamepad1.left_stick_y);
            flywheel.setPower(gamepad1.right_stick_y);
            index.setPower(gamepad1.a ? 1 : 0);
        }
    }
}
