package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;

@TeleOp (group="UnitTest")
public class TurretTest extends LinearOpMode {
    Turret turret;
    Follower drive;

    boolean tOnPressed = false, tOffPressed = false;
    boolean targetPressed = false;
    @Override
    public void runOpMode() {
        drive = Constants.createFollower(hardwareMap);
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

            if (gamepad1.x && !targetPressed)
                turret.setYaw(-Math.PI/2);
            targetPressed = gamepad1.x;

            if (gamepad1.y) turret.face(FieldPoses.redHoop, drive.getPose());

            turret.periodic();
            drive.updatePose();
            telemetry.addData("Turret Angle",turret.getYaw());
            telemetry.addData("Get Turret", turret.getTurret());
            telemetry.addData("Turret Target", turret.getTurretTarget());
            telemetry.update();
        }
    }
}
