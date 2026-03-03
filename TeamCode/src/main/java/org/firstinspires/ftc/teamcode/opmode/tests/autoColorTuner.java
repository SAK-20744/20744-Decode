package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.subsystems.BallSensors2;
import org.firstinspires.ftc.teamcode.util.FileConfig;
import org.opencv.core.Scalar;

@TeleOp
public class autoColorTuner extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    ApolloConstants.colorRanges colorRanges;
    BallSensors2 sensors;
    String mode = "solid";
    @Override
    public void runOpMode() {
        colorRanges = new ApolloConstants.colorRanges();
        sensors = new BallSensors2(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) mode = "solid";
            if (gamepad1.dpad_down) mode = "hole";
            NormalizedRGBA left = sensors.l1.getNormalizedColors();
            NormalizedRGBA mid = sensors.m1.getNormalizedColors();
            NormalizedRGBA right = sensors.r1.getNormalizedColors();

            if (gamepad1.xWasPressed()) save_left(left);
            if (gamepad1.yWasPressed()) save_mid(mid);
            if (gamepad1.bWasPressed()) save_right(right);

            telemetry.addData("left ", "%.3f %.3f %.3f", left.red,left.green,left.blue);
            telemetry.addData("mid  ", "%.3f %.3f %.3f", mid.red,mid.green,mid.blue);
            telemetry.addData("right", "%.3f %.3f %.3f", right.red,right.green,right.blue);
            telemetry.addData("Mode",mode);
            telemetry.addLine();
            telemetry.addLine("Press one of the following buttons to save the values to the corresponding sensor:\n");
            telemetry.addLine("X: Left\nY: Middle\nB: Right");
            telemetry.addLine("DpadUp: Solid  |  DpadDown: Hole");
            telemetry.update();


        }
        FileConfig.saveJson("ColorSensorRanges.json", colorRanges);
        FileConfig.writeFile("endProgramTest.txt", "ended with: "+getRuntime());
    }
    public void save_left(NormalizedRGBA rgb) {
        if (mode == "solid") colorRanges.lGMax = new Scalar(rgb.red,rgb.green,rgb.blue);
        else if (mode == "hole") colorRanges.lGMin = new Scalar(rgb.red,rgb.green,rgb.blue);
    }
    public void save_mid(NormalizedRGBA rgb) {
        if (mode == "solid") colorRanges.mGMax = new Scalar(rgb.red,rgb.green,rgb.blue);
        else if (mode == "hole") colorRanges.mGMin = new Scalar(rgb.red,rgb.green,rgb.blue);
    }
    public void save_right(NormalizedRGBA rgb) {
        if (mode == "solid") colorRanges.rGMax = new Scalar(rgb.red,rgb.green,rgb.blue);
        else if (mode == "hole") colorRanges.rGMin = new Scalar(rgb.red,rgb.green,rgb.blue);
    }
}
