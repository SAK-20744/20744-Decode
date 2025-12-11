package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp (group="UnitTest")
public class TurretTest extends LinearOpMode {
    Turret turret;

    boolean tOnPressed = false, tOffPressed = false;
    @Override
    public void runOpMode() {
        turret = new Turret(hardwareMap);
        turret.off();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && !tOnPressed)
                turret.on();
            tOnPressed = gamepad1.a;
            if (gamepad1.b && !tOffPressed)
                turret.off();
            tOffPressed = gamepad1.b;

            turret.periodic();

            telemetry.addData("Turret Angle",turret.getYaw());
            telemetry.addData("Get Turret", turret.getTurret());
            telemetry.addData("Turret Target", turret.getTurretTarget());
            telemetry.update();
        }
    }
}
