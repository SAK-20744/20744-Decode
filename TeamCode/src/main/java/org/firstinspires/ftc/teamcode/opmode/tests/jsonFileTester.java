package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.FileConfig;

@Disabled
@TeleOp
public class jsonFileTester extends LinearOpMode {
    // Example Usage for Json File Reading and Writing

    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());
        waitForStart();
        if (opModeIsActive()) {
            FileConfig.saveJson("configTest.json", new Data("Testing", 100d));
            Data data = FileConfig.loadJson("configTest.json", Data.class);
            telemetry.addData("var 1", data.variable1);
            telemetry.addData("var 2", data.variable2);
            telemetry.update();
            while (opModeIsActive());
        }
    }
    private static class Data {
        String variable1;
        double variable2;

        Data(String variable1, double variable2) {
            this.variable1 = variable1;
            this.variable2 = variable2;
        }
    }
}
