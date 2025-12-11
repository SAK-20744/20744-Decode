package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kickers;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;

@TeleOp
public class FullShooterTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Shooter shooter;
    Follower drive;
    Intake intake;
    Kickers kickers;

    boolean farPressed = false, closePressed = false, onPressed = false, offPressed = false;

    boolean lPressed=false,mPressed=false,rPressed=false;
    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap);
        drive = Constants.createFollower(hardwareMap);
        kickers = new Kickers(hardwareMap);
        intake = new Intake(hardwareMap);


        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_left && !lPressed)
                kickers.kick(Kickers.Kicker.L);
            lPressed = gamepad1.dpad_left;

            if (gamepad1.dpad_up && !mPressed)
                kickers.kick(Kickers.Kicker.M);
            mPressed = gamepad1.dpad_up;

            if (gamepad1.dpad_right && !rPressed)
                kickers.kick(Kickers.Kicker.R);
            rPressed = gamepad1.dpad_right;

            kickers.periodic();

            if (gamepad1.right_bumper)
                intake.spinIn();
            else if (gamepad1.left_bumper)
                intake.spinOut();
            else
                intake.spinIdle();


            if (gamepad1.a && !closePressed) {
                shooter.close(); shooter.down();
            }closePressed = gamepad1.a;
            if (gamepad1.b && !farPressed) {
                shooter.far(); shooter.up();
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
