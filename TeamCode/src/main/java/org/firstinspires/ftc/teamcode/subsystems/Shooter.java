package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private DcMotorEx lShooter, rShooter;
    private DcMotor turret;
    private Servo hood;

    public Shooter (HardwareMap hardwareMap) {
        lShooter = hardwareMap.get(DcMotorEx.class, "lShooter");
        rShooter = hardwareMap.get(DcMotorEx.class, "rShooter");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        hood = hardwareMap.get(Servo.class, "hood");
    }
    public void update() {
        lShooter.setPower(rShooter.getPower());
    }
}
