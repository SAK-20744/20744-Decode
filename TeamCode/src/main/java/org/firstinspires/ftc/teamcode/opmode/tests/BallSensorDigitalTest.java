package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BallSensorsDigital;
import org.firstinspires.ftc.teamcode.util.Motif;

import java.util.Arrays;

@TeleOp (group="UnitTest")
public class BallSensorDigitalTest extends LinearOpMode {
    Motif motif = Motif.PPG;
    BallSensorsDigital bsd;

    boolean cyclePressed = false;
    @Override
    public void runOpMode() {
        bsd = new BallSensorsDigital(hardwareMap);
        bsd.motif(motif);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Controls:");
        telemetry.addLine("<A> -> Cycle Motif");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && !cyclePressed) cycleMotif();
            cyclePressed = gamepad1.a;

            bsd.read();

            telemetry.addLine("--Colors--");
            telemetry.addData("left Color", bsd.leftC());
            telemetry.addData("middle Color", bsd.middleC());
            telemetry.addData("right Color", bsd.rightC());
            telemetry.addLine("--In Slot--");
            telemetry.addData("left", bsd.leftD());
            telemetry.addData("middle", bsd.middleD());
            telemetry.addData("right", bsd.rightD());
            telemetry.addLine("--Shoot Order--");
            telemetry.addData("Motif", motif);
            telemetry.addData("Shoot Sequence", Arrays.toString(bsd.shootSequence()) );
            telemetry.update();
        }
    }
    public void cycleMotif() {
        switch (motif) {
            case GPP: motif = Motif.PGP;break;
            case PGP: motif = Motif.PPG;break;
            case PPG: motif = Motif.GPP;break;
        }
        bsd.motif(motif);
    }
}
