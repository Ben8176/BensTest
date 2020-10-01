package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PrototypeRobot {

    public DcMotorEx flywheel, intake;
    public Servo index;

    public PrototypeRobot(HardwareMap hwmap) {
        flywheel = hwmap.get(DcMotorEx.class, "shooter");
        intake = hwmap.get(DcMotorEx.class, "intake");
        index = hwmap.get(Servo.class, "index");

    }
}
