package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BallSensors;
import org.firstinspires.ftc.teamcode.util.Motif;

import java.util.Arrays;

@TeleOp (group="UnitTest")
public class BallSensorTest extends LinearOpMode {
    Motif motif = Motif.PPG;
    BallSensors balls;

    boolean cyclePressed = false;
    @Override
    public void runOpMode() {
        balls = new BallSensors(hardwareMap);
        balls.motif(motif);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Controls:");
        telemetry.addLine("<A> -> Cycle Motif");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && !cyclePressed) cycleMotif();
            cyclePressed = gamepad1.a;

            balls.periodic();

            telemetry.addLine("--Colors--");
            telemetry.addData("left Color", balls.leftC());
            telemetry.addData("middle Color", balls.middleC());
            telemetry.addData("right Color", balls.rightC());
            telemetry.addLine("--In Slot--");
            telemetry.addData("left", balls.leftD());
            telemetry.addData("middle", balls.middleD());
            telemetry.addData("right", balls.rightD());
            telemetry.addLine("--Shoot Order--");
            telemetry.addData("Motif", motif);
            telemetry.addData("Shoot Sequence", Arrays.toString(balls.shootSequence()) );
            telemetry.update();
        }
    }
    public void cycleMotif() {
        switch (motif) {
            case GPP: motif = Motif.PGP;break;
            case PGP: motif = Motif.PPG;break;
            case PPG: motif = Motif.GPP;break;
        }
        balls.motif(motif);
    }
}
