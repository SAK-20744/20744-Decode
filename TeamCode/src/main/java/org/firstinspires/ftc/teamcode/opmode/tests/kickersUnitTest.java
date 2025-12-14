package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Kickers;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


@TeleOp (group="UnitTest")
public class kickersUnitTest extends LinearOpMode {
    boolean lPressed=false,mPressed=false,rPressed=false;

    RevColorSensorV3 m1,m2, l1,l2,r1,r2;
    boolean left, mid, right = false;


    @Override
    public void runOpMode() {
        Kickers kickers = new Kickers(hardwareMap);

        m1 = hardwareMap.get(RevColorSensorV3.class, "m1");
        l1 = hardwareMap.get(RevColorSensorV3.class, "l1");
        l2 = hardwareMap.get(RevColorSensorV3.class, "l2");
        r1 = hardwareMap.get(RevColorSensorV3.class, "r1");
        r2 = hardwareMap.get(RevColorSensorV3.class, "r2");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x && !lPressed)
                kickers.kick(Kickers.Kicker.L);
            lPressed = gamepad1.x;

            if (gamepad1.y && !mPressed)
                kickers.kick(Kickers.Kicker.M);
            mPressed = gamepad1.y;

            if (gamepad1.b && !rPressed)
                kickers.kick(Kickers.Kicker.R);
            rPressed = gamepad1.b;

            if(l1.getDistance(DistanceUnit.MM) < 30 || l2.getDistance(DistanceUnit.MM) < 25)
                left=true;
            else left = false;

            if(r1.getDistance(DistanceUnit.MM) < 30 || r2.getDistance(DistanceUnit.MM) < 25)
                right=true;
            else right = false;

            if(m1.getDistance(DistanceUnit.MM) < 34)
                mid=true;
            else mid = false;

            kickers.periodic();

            telemetry.addData("Left?", left);
            telemetry.addData("Right?", right);
            telemetry.addData("Middle?", mid);

            telemetry.addData("m1 Dist", m1.getDistance(DistanceUnit.MM));

            telemetry.addData("L1 Red", "%.3f", l1.getNormalizedColors().red);
            telemetry.addData("L1 Green", "%.3f", l1.getNormalizedColors().green);
            telemetry.addData("L1 Blue", "%.3f", l1.getNormalizedColors().blue);

            telemetry.addData("L2 Red", "%.3f", l2.getNormalizedColors().red);
            telemetry.addData("L2 Green", "%.3f", l2.getNormalizedColors().green);
            telemetry.addData("L2 Blue", "%.3f", l2.getNormalizedColors().blue);

//            telemetry.addData("Kicker Up",kickers.kickerUp());
//            telemetry.addData("Kicker Down",kickers.kickerDown());
//            telemetry.addData("Get Kicker Up", kickers.getUp());
//            telemetry.addData("Get Queued", kickers.getQueued());
            telemetry.update();
        }
    }
}
