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

@TeleOp (group="UnitTest")
public class autoColorTuner extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private ApolloConstants.colorRanges colorRanges;
    private BallSensors2 sensors;
    public String mode = "solid";
    public double errorPercent = 15;
    private double minusMulti = 0.85, plusMulti = 1.15;
    @Override
    public void runOpMode() {
        colorRanges = new ApolloConstants.colorRanges();
        sensors = new BallSensors2(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) mode = "solid";
            if (gamepad1.dpad_down) mode = "hole";

            if (gamepad1.leftBumperWasPressed()) errorPercent -= 5;
            if (gamepad1.rightBumperWasPressed()) errorPercent += 5;

            double errorDecimal = errorPercent/100;
            minusMulti = 1 - errorDecimal;
            plusMulti = 1 + errorDecimal;

            NormalizedRGBA left = sensors.l1.getNormalizedColors();
            NormalizedRGBA mid = sensors.m1.getNormalizedColors();
            NormalizedRGBA right = sensors.r1.getNormalizedColors();

            NormalizedRGBA left2 = sensors.l2.getNormalizedColors();
            NormalizedRGBA mid2 = sensors.m2.getNormalizedColors();
            NormalizedRGBA right2 = sensors.r2.getNormalizedColors();

            if (gamepad1.xWasPressed()) { save_left(left); save_left2(left2); }
            if (gamepad1.yWasPressed()) { save_mid(mid); save_mid2(mid2); }
            if (gamepad1.bWasPressed()) { save_right(right); save_right2(right2); }

            telemetry.addData("left ", "%.3f %.3f %.3f", left.red,left.green,left.blue);
            telemetry.addData("mid  ", "%.3f %.3f %.3f", mid.red,mid.green,mid.blue);
            telemetry.addData("right", "%.3f %.3f %.3f", right.red,right.green,right.blue);
            telemetry.addData("left2 ", "%.3f %.3f %.3f", left2.red,left2.green,left2.blue);
            telemetry.addData("mid2  ", "%.3f %.3f %.3f", mid2.red,mid2.green,mid2.blue);
            telemetry.addData("right2", "%.3f %.3f %.3f", right2.red,right2.green,right2.blue);
            telemetry.addData("Mode",mode);
            telemetry.addData("Error Percent", errorPercent);
            telemetry.addLine();
            telemetry.addLine("Press one of the following buttons to save the values to the corresponding sensor:\n");
            telemetry.addLine("X: Left\nY: Middle\nB: Right");
            telemetry.addLine("DpadUp: Solid  |  DpadDown: Hole");
            telemetry.addLine("Error Percent Up/Down : LeftBumper / RightBumper");
            telemetry.update();


        }
        FileConfig.saveJson("ColorSensorRanges.json", colorRanges);
        FileConfig.writeFile("endProgramTest.txt", "ended with: "+getRuntime());
    }
    public void save_left(NormalizedRGBA rgb) {
        if (mode == "solid") colorRanges.lGMax = new Scalar(rgb.red*plusMulti,rgb.green*plusMulti,rgb.blue*plusMulti);
        else if (mode == "hole") colorRanges.lGMin = new Scalar(rgb.red*minusMulti,rgb.green*minusMulti,rgb.blue*minusMulti);
    }
    public void save_mid(NormalizedRGBA rgb) {
        if (mode == "solid") colorRanges.mGMax = new Scalar(rgb.red*plusMulti,rgb.green*plusMulti,rgb.blue*plusMulti);
        else if (mode == "hole") colorRanges.mGMin = new Scalar(rgb.red*minusMulti,rgb.green*minusMulti,rgb.blue*minusMulti);
    }
    public void save_right(NormalizedRGBA rgb) {
        if (mode == "solid") colorRanges.rGMax = new Scalar(rgb.red*plusMulti,rgb.green*plusMulti,rgb.blue*plusMulti);
        else if (mode == "hole") colorRanges.rGMin = new Scalar(rgb.red*minusMulti,rgb.green*minusMulti,rgb.blue*minusMulti);
    }
    public void save_left2(NormalizedRGBA rgb) {
        if (mode == "solid") colorRanges.l2GMax = new Scalar(rgb.red*plusMulti,rgb.green*plusMulti,rgb.blue*plusMulti);
        else if (mode == "hole") colorRanges.l2GMin = new Scalar(rgb.red*minusMulti,rgb.green*minusMulti,rgb.blue*minusMulti);
    }
    public void save_mid2(NormalizedRGBA rgb) {
        if (mode == "solid") colorRanges.m2GMax = new Scalar(rgb.red*plusMulti,rgb.green*plusMulti,rgb.blue*plusMulti);
        else if (mode == "hole") colorRanges.m2GMin = new Scalar(rgb.red*minusMulti,rgb.green*minusMulti,rgb.blue*minusMulti);
    }
    public void save_right2(NormalizedRGBA rgb) {
        if (mode == "solid") colorRanges.r2GMax = new Scalar(rgb.red*plusMulti,rgb.green*plusMulti,rgb.blue*plusMulti);
        else if (mode == "hole") colorRanges.r2GMin = new Scalar(rgb.red*minusMulti,rgb.green*minusMulti,rgb.blue*minusMulti);
    }
}
