package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class ApolloConstants {

    public static double autoTurret = 115;
    public static double autoTurret2 = 111;
    public static double autoTurret3 = 108;

    public static double blueautoTurret = -115;
    public static double blueautoTurret2 = -118;
    public static double blueautoTurret3 = -121;

    public static double intakeMovementSpeed = 0.35;

    public static double
        INTAKE_IN = 1,
        INTAKE_OUT = -1,
        INTAKE_OFF = 0,
        LKICKER_DOWN = 0,
        MKICKER_DOWN = 0.01,
        RKICKER_DOWN = 0,
        LKICKER_UP = 0.27,
        MKICKER_UP = 0.27,
        RKICKER_UP = 0.27,
        HOOD_CLOSE = 0.3,
        HOOD_FAR = 0.8,
        SHOOTER_CLOSE = 0.65,
        SHOOTER_FAR = 0.8,
        SHOOTER_OFF = 0,
        VELOCITY_FAR = 480,
        VELOCITY_CLOSE = 330,
        tpl = 0.001,
        til = 0,
        tdl = 0.00013,
        tps = 0.0007,
        tis = 0,
        tds = 0.000000075,
        TURRET_THRESHOLD = 100,
        OFFSET = 650;

    public static double shooterkS = 0.135, shooterkV = 0.000525, shooterkP = 0.02;


    // OWENS CODE. ONLY FOR FULL SHOOTER TEST
    public static double
        KICKER_UPTIME = 0.5,
        KICKER_DOWNTIME = 0.3;

    public static double
        KUP = 900,
        KDOWN = 150;
    public static int
        TURRET_BLUE_FAR = -285,
        TURRET_MIDDLE = 0,
        TURRET_RED_FAR = 285;
    public static DcMotorSimple.Direction
        frDir = DcMotorSimple.Direction.FORWARD,
        brDir = DcMotorSimple.Direction.FORWARD,
        flDir = DcMotorSimple.Direction.REVERSE,
        blDir = DcMotorSimple.Direction.REVERSE,
        turretDir = DcMotorSimple.Direction.FORWARD,
        intakeDir = DcMotorSimple.Direction.REVERSE,
        lShooterDir = DcMotorSimple.Direction.REVERSE,
        rShooterDir = DcMotorSimple.Direction.FORWARD;
    public static class dt {
        //drivetrain
        public static String
                fl = "fl",
                fr = "fr",
                bl = "bl",
                br = "br";
    }
    public static class CS {
        public static double error = 0.008;
        public static class G {
            public static double l1R = 0.005, l1G = 0.019, l1B = 0.014;
            public static double l2R = 0.006, l2G = 0.025, l2B = 0.018;
        }
        public static class P {
            public static double l1R = 0.011, l1G = 0.013, l1B = 0.020;
            public static double l2R = 0.015, l2G = 0.018, l2B = 0.028;
        }
    }
}