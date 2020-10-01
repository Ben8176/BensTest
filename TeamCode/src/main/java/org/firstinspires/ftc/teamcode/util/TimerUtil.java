package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TimerUtil {

    ElapsedTime timer = new ElapsedTime();

    public TimerUtil() {}

    public void startTimer() {
        timer.reset();
    }

    public double getMillis() {
       return timer.milliseconds();
    }

    public double getSeconds() {
        return timer.seconds();
    }
}
