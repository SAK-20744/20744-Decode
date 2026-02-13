package org.firstinspires.ftc.teamcode.opmode.tests;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Configurable

@TeleOp(group="UnitTest")
public class LightFade extends LinearOpMode {

    Servo light;
    ElapsedTime lightTimer = new ElapsedTime();

    // goBILDA RGB Indicator positions
//    final double RED = 0.277;
//    final double PURPLE = 0.722;

    double looptime = 0;

    // seconds to go from red -> purple (and same time back)
//    public static double UP_TIME = 2.0;

    @Override
    public void runOpMode() {
        light = hardwareMap.get(Servo.class, "light");

        waitForStart();
        lightTimer.reset();

        while (opModeIsActive()) {

            double t = lightTimer.seconds();

            // triangle wave: 0..1..0 repeating every 2*UP_TIME seconds
            double phase = (t % (2.0 * UP_TIME)) / UP_TIME; // 0..2
            double tri = (phase <= 1.0) ? phase : (2.0 - phase); // 0..1..0

            // map triangle wave into servo position range
            double pos = REDLIGHT + tri * (PURPLELIGHT - REDLIGHT);

            light.setPosition(pos);

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - looptime));
            looptime = loop;
            telemetry.update();

            // keep running your robot code here (no sleep)
        }
    }
}
