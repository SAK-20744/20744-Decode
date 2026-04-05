package org.firstinspires.ftc.teamcode.opmode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class BrushlandsDigitalTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DigitalChannel leftPurple = hardwareMap.digitalChannel.get("leftpurple");
        DigitalChannel leftGreen = hardwareMap.digitalChannel.get("leftgreen");
        DigitalChannel middlePurple = hardwareMap.digitalChannel.get("middlepurple");
        DigitalChannel middleGreen = hardwareMap.digitalChannel.get("middlegreen");
        DigitalChannel rightPurple = hardwareMap.digitalChannel.get("rightpurple");
        DigitalChannel rightGreen = hardwareMap.digitalChannel.get("rightgreen");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("left purple", leftPurple.getState());
            telemetry.addData("left green", leftGreen.getState());
            telemetry.addData("middle purple", middlePurple.getState());
            telemetry.addData("middle green", middleGreen.getState());
            telemetry.addData("right purple", rightPurple.getState());
            telemetry.addData("right green", rightGreen.getState());
            telemetry.update();
        }

    }

}
