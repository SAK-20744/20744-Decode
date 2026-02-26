package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.FileConfig;
import org.opencv.core.Scalar;

@Disabled
@TeleOp
public class scalarJsonTester extends LinearOpMode {
    // Example Usage for Json File Reading and Writing

    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());
        waitForStart();
        if (opModeIsActive()) {
            FileConfig.saveJson("configTest.json", new Data("Testing", 100d, new Scalar(100,200,300)));
            Data data = FileConfig.loadJson("configTest.json", Data.class);
            telemetry.addData("var 1", data.variable1);
            telemetry.addData("var 2", data.variable2);
            telemetry.addData("var 3", data.variable3);
            telemetry.update();
            while (opModeIsActive());
        }
    }
    private static class Data {
        String variable1;
        double variable2;
        Scalar variable3;

        Data(String variable1, double variable2, Scalar variable3) {
            this.variable1 = variable1;
            this.variable2 = variable2;
            this.variable3 = variable3;
        }
    }
}
