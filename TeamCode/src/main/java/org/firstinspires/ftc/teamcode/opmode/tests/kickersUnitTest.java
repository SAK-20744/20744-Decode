package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Kickers;


@TeleOp
public class kickersUnitTest extends LinearOpMode {
    boolean lPressed=false,mPressed=false,rPressed=false;
    @Override
    public void runOpMode() {
        Kickers kickers = new Kickers(hardwareMap);
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

            kickers.periodic();

            telemetry.addData("Kicker Up",kickers.kickerUp());
            telemetry.addData("Kicker Down",kickers.kickerDown());
            telemetry.addData("Get Kicker Up", kickers.getUp());
            telemetry.addData("Get Queued", kickers.getQueued());
            telemetry.update();
        }
    }
}
