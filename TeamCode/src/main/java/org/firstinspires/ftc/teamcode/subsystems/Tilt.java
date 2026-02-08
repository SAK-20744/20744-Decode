package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tilt {
    Servo t1, t2;
    boolean extended = false;
    public Tilt(HardwareMap hw) {
        t1 = hw.get(Servo.class, "lift1");
        t2 = hw.get(Servo.class, "lift2");
    }
    public void retract() {
        t1.setPosition(TILT_RETRACT);
        t2.setPosition(TILT_RETRACT);
        extended = false;
    }
    public void extend() {
        t1.setPosition(TILT_EXTEND);
        t2.setPosition(TILT_EXTEND);
        extended = true;
    }
    public void toggle() {
        if (extended) retract();
        else extend();
    }
}
