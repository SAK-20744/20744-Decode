package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LLAutoAim extends OpMode {
    DcMotor turretMotor;
    Limelight3A limelight;
    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

    }
    @Override
    public void start() {
        limelight.start();
    }
    @Override
    public void loop() {
        double tx = limelight.getLatestResult().getTx();
        turretMotor.setPower(tx/22.5);
    }
}

