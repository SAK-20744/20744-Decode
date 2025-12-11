package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class JustShooterPID extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Shooter shooter;

    boolean farPressed = false, closePressed = false, onPressed = false, offPressed = false;
    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && !closePressed) {
                shooter.close(); shooter.up();
            }closePressed = gamepad1.a;
            if (gamepad1.b && !farPressed) {
                shooter.far(); shooter.down();
            }farPressed = gamepad1.b;
            if (gamepad1.x && !onPressed)
                shooter.on();
            onPressed = gamepad1.x;
            if (gamepad1.y && !offPressed)
                shooter.off();
            offPressed = gamepad1.y;

            shooter.periodic();

            telemetry.addData("Shooter Vel", shooter.getVelocity());
            telemetry.addData("Shooter Target Vel", shooter.getTarget());
            telemetry.addData("Shooter Activated", shooter.isActivated());
            telemetry.update();
        }
    }
}
