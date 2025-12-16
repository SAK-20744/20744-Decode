package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.subsystems.Kickers;
import org.firstinspires.ftc.teamcode.util.BallColor;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


@TeleOp (group="UnitTest")
public class kickersUnitTest extends LinearOpMode {
    boolean cyclePressed = false;
    boolean lPressed=false,mPressed=false,rPressed=false;

    RevColorSensorV3 m1,m2, l1,l2,r1,r2;
    boolean left, mid, right = false;

    double colorError= 0.005;
    double sensor1R, sensor1G, sensor1B;
    double sensor2R, sensor2G, sensor2B;

    BallColor sensor1Color = BallColor.N,sensor2Color = BallColor.N;
    String selectedSensor = "L";
    RevColorSensorV3 sensor1 = l1,sensor2 = l2;

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

            if (gamepad1.a && !cyclePressed)
                cycleSensor();
            cyclePressed = gamepad1.a;

            if(l1.getDistance(DistanceUnit.MM) < 30 || l2.getDistance(DistanceUnit.MM) < 25)
                left=true;
            else left = false;

            if(r1.getDistance(DistanceUnit.MM) < 30 || r2.getDistance(DistanceUnit.MM) < 25)
                right=true;
            else right = false;

            if(m1.getDistance(DistanceUnit.MM) < 34)
                mid=true;
            else mid = false;

            if (selectedSensor == "L") {sensor1 = l1;sensor2 = l2;}
            if (selectedSensor == "M") {sensor1 = m1;sensor2 = m1;}
            if (selectedSensor == "R") {sensor1 = r1;sensor2 = r2;}

            sensor1R = sensor1.getNormalizedColors().red;
            sensor1G = sensor1.getNormalizedColors().green;
            sensor1B = sensor1.getNormalizedColors().blue;

            sensor2R = sensor2.getNormalizedColors().red;
            sensor2G = sensor2.getNormalizedColors().green;
            sensor2B = sensor2.getNormalizedColors().blue;

            if (inCRange(sensor1R,sensor1G,sensor1B, ApolloConstants.CS.G.l1R,ApolloConstants.CS.G.l1G,ApolloConstants.CS.G.l1B))
                sensor1Color = BallColor.G;
            else if (inCRange(sensor1R,sensor1G,sensor1B,ApolloConstants.CS.P.l1R,ApolloConstants.CS.P.l1G,ApolloConstants.CS.P.l1B))
                sensor1Color = BallColor.P;
            else
                sensor1Color = BallColor.N;

            if (inCRange(sensor2R,sensor2G,sensor2B, ApolloConstants.CS.G.l2R,ApolloConstants.CS.G.l2G,ApolloConstants.CS.G.l2B))
                sensor2Color = BallColor.G;
            else if (inCRange(sensor2R,sensor2G,sensor2B,ApolloConstants.CS.P.l2R,ApolloConstants.CS.P.l2G,ApolloConstants.CS.P.l2B))
                sensor2Color = BallColor.P;
            else
                sensor2Color = BallColor.N;

            kickers.periodic();

            telemetry.addData("Sensor Selected",selectedSensor);
            telemetry.addData("Left?", left);
            telemetry.addData("Right?", right);
            telemetry.addData("Middle?", mid);

            telemetry.addData("m1 Dist", m1.getDistance(DistanceUnit.MM));

            telemetry.addData("sensor1 Red", "%.3f", sensor1.getNormalizedColors().red);
            telemetry.addData("sensor1 Green", "%.3f", sensor1.getNormalizedColors().green);
            telemetry.addData("sensor1 Blue", "%.3f", sensor1.getNormalizedColors().blue);

            telemetry.addData("sensor2 Red", "%.3f", sensor2R);
            telemetry.addData("sensor2 Green", "%.3f", sensor2G);
            telemetry.addData("sensor2 Blue", "%.3f", sensor2B);

            switch (sensor1Color) {
                case G: telemetry.addLine("sensor1 is green"); break;
                case P: telemetry.addLine("sensor1 is purple"); break;
                case N: telemetry.addLine("sensor1 no ball"); break;
            }

            switch (sensor2Color) {
                case G: telemetry.addLine("sensor2 is green"); break;
                case P: telemetry.addLine("sensor2 is purple"); break;
                case N: telemetry.addLine("sensor2 no ball"); break;
            }

//            telemetry.addData("Kicker Up",kickers.kickerUp());
//            telemetry.addData("Kicker Down",kickers.kickerDown());
//            telemetry.addData("Get Kicker Up", kickers.getUp());
//            telemetry.addData("Get Queued", kickers.getQueued());
            telemetry.update();
        }
    }
    boolean inCRange(double r,double g,double b, double tr, double tg, double tb) {
        return (Math.abs(tr-r) < colorError && Math.abs(tg-g) < colorError && Math.abs(tb-b) < colorError);
    }
    void cycleSensor() {
        switch (selectedSensor) {
            case "L": selectedSensor = "M";break;
            case "M": selectedSensor = "R";break;
            case "R": selectedSensor = "L";break;
        }
    }
}
