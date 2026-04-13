package org.firstinspires.ftc.teamcode.opmode.tests;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.BallColor;


@TeleOp (group="UnitTest")
public class manualBrushlandsTuner extends LinearOpMode {
    boolean cyclePressed = false;

    RevColorSensorV3 m1,l1,r1;
    boolean left, mid, right = false;

    double sensor1R, sensor1G, sensor1B;

    BallColor sensor1Color = BallColor.N;
    String selectedSensor = "L";
    RevColorSensorV3 sensor1 = l1;

    @Override
    public void runOpMode() {
        Intake intake = new Intake(hardwareMap);

        m1 = hardwareMap.get(RevColorSensorV3.class, "r2");
        l1 = hardwareMap.get(RevColorSensorV3.class, "m2");
        r1 = hardwareMap.get(RevColorSensorV3.class, "m1");

        l1.setGain(50);
        m1.setGain(50);
        r1.setGain(50);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.right_bumper)
                intake.spinIn();
            else if (gamepad1.left_bumper) {
                intake.spinOut();
            } else {
                intake.idle();
            }

            if (gamepad1.a && !cyclePressed)
                cycleSensor();
            cyclePressed = gamepad1.a;

            if(l1.getDistance(DistanceUnit.MM) < 35)
                left=true;
            else left = false;

            if(r1.getDistance(DistanceUnit.MM) < 35)
                right=true;
            else right = false;

            if(m1.getDistance(DistanceUnit.MM) < 35)
                mid=true;
            else mid = false;

            if (selectedSensor == "L") {sensor1 = l1;}
            if (selectedSensor == "M") {sensor1 = m1;}
            if (selectedSensor == "R") {sensor1 = r1;}

            sensor1R = sensor1.getNormalizedColors().red;
            sensor1G = sensor1.getNormalizedColors().green;
            sensor1B = sensor1.getNormalizedColors().blue;

            final float[] hsvValues = new float[3];
            Color.colorToHSV(sensor1.getNormalizedColors().toColor(), hsvValues);

            if (inCRange(sensor1R,sensor1G,sensor1B, ApolloConstants.CS.G.m1R,ApolloConstants.CS.G.m1G,ApolloConstants.CS.G.m1B))
                sensor1Color = BallColor.G;
            else if (inCRange(sensor1R,sensor1G,sensor1B,ApolloConstants.CS.P.m1R,ApolloConstants.CS.P.m1G,ApolloConstants.CS.P.m1B))
                sensor1Color = BallColor.P;
            else
                sensor1Color = BallColor.N;


            telemetry.addData("Sensor Selected",selectedSensor);
            telemetry.addData("Left?", left);
            telemetry.addData("Right?", right);
            telemetry.addData("Middle?", mid);

            telemetry.addData("s1 Dist", sensor1.getDistance(DistanceUnit.MM));

            telemetry.addData("sensor1 Red", "%.3f", sensor1R);
            telemetry.addData("sensor1 Green", "%.3f", sensor1G);
            telemetry.addData("sensor1 Blue", "%.3f", sensor1B);
            telemetry.addLine();

            telemetry.addData("sensor1 Hue", "%.3f", hsvValues[0]);
            telemetry.addData("sensor1 Saturation", "%.3f", hsvValues[1]);
            telemetry.addData("sensor1 Value", "%.3f", hsvValues[2]);

//            switch (sensor1Color) {
//                case G: telemetry.addLine("sensor1 is green"); break;
//                case P: telemetry.addLine("sensor1 is purple"); break;
//                case N: telemetry.addLine("sensor1 no ball"); break;
//            }
//
//            switch (sensor2Color) {
//                case G: telemetry.addLine("sensor2 is green"); break;
//                case P: telemetry.addLine("sensor2 is purple"); break;
//                case N: telemetry.addLine("sensor2 no ball"); break;
//            }

//            telemetry.addData("Kicker Up",kickers.kickerUp());
//            telemetry.addData("Kicker Down",kickers.kickerDown());
//            telemetry.addData("Get Kicker Up", kickers.getUp());
//            telemetry.addData("Get Queued", kickers.getQueued());
            telemetry.update();
        }
    }
    boolean inCRange(double r,double g,double b, double tr, double tg, double tb) {
        return (Math.abs(tr-r) < ApolloConstants.CS.m1RRange && Math.abs(tg-g) < ApolloConstants.CS.m1RRange && Math.abs(tb-b) < ApolloConstants.CS.m1RRange);
    }
    void cycleSensor() {
        switch (selectedSensor) {
            case "L": selectedSensor = "M";break;
            case "M": selectedSensor = "R";break;
            case "R": selectedSensor = "L";break;
        }
    }
}
