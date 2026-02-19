package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class ApolloConstants {

    public static double autoTurret = 115;
    public static double autoTurret2 = 115;
    public static double autoTurret3 = 115;

    public static double blueautoTurret = -116;
    public static double blueautoTurret2 = -116;
    public static double blueautoTurret3 = -116;

    public static double intakeMovementSpeed = 0.35;

    public static double shooterVelError = 40;

    public static double
        INTAKE_IN = 1,
        INTAKE_OUT = -1,
        INTAKE_OFF = 0,
        LKICKER_DOWN = 0,
        MKICKER_DOWN = 0.01,
        RKICKER_DOWN = 0,
        LKICKER_UP = 0.28,
        MKICKER_UP = 0.28,
        RKICKER_UP = 0.28,
        HOOD_MIN = 0.07,
        HOOD_MAX = 0.72,
        HOOD_CLOSE = 0.22,
        HOOD_FAR = 0.68,
//        SHOOTER_CLOSE = 0.65,
//        SHOOTER_FAR = 0.8,
//        SHOOTER_OFF = 0,
        VELOCITY_FAR = 480,
        VELOCITY_CLOSE = 330,
        tpl = 0.001,
        til = 0,
        tdl = 0.00013,
        tps = 0.0007,
        tis = 0,
        tds = 0.000000075,
        TURRET_THRESHOLD = 100,
        OFFSET = 650,
        TILT_RETRACT = 0.56,
        TILT_EXTEND = 0.35;

    public static double shooterkS = 0.135, shooterkV = 0.000525, shooterkP = 0.02;

    public static double REDLIGHT = 0.277;
    public static double PURPLELIGHT = 0.722;
    public static double UP_TIME = 5.0;

    public static double CAM_FWD_IN  = 5.0;   // camera offset forward from turret center
    public static double CAM_LEFT_IN = 0.5;   // camera offset left from turret center

    public static double VISION_MAX_JUMP_IN = 1; // inches
    public static double VISION_ALPHA = 0.075;       // blend factor



//    // OWENS CODE. ONLY FOR FULL SHOOTER TEST
//    public static double
//        KICKER_UPTIME = 0.5,
//        KICKER_DOWNTIME = 0.3;

    public static double
        KUP = 200,
        KDOWN = 100;
    public static int
//        TURRET_BLUE_FAR = -285,
        TURRET_MIDDLE = 0;
//        TURRET_RED_FAR = 285;
    public static DcMotorSimple.Direction
        frDir = DcMotorSimple.Direction.FORWARD,
        brDir = DcMotorSimple.Direction.FORWARD,
        flDir = DcMotorSimple.Direction.REVERSE,
        blDir = DcMotorSimple.Direction.REVERSE,
        turretDir = DcMotorSimple.Direction.FORWARD,
        intakeDir = DcMotorSimple.Direction.FORWARD,
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
        //      Left Sensor:
        // NoBall:      R=0.185 G=0.311 B=0.285
        // HolePurple:  R=0.213 G=0.336 B=0.339
        // Purple:      R=0.222 G=0.347 B=0.356
        // HoleGreen:   R=0.195 G=0.365 B=0.336
        // Green:       R=0.197 G=0.379 B=0.347
        //      Middle Sensor:
        // NoBall:      R=0.189 G=0.330 B=0.291
        // HolePurple:  R=0.275 G=0.401 B=0.450
        // Purple:      R=0.299 G=0.438 B=0.470
        // HoleGreen:   R=0.204 G=0.479 B=0.380
        // Green:       R=0.216 G=0.527 B=0.409
        //      Right Sensor:
        // NoBall:      R=0.191 G=0.351 B=0.328
        // HolePurple:  R=0.267 G=0.412 B=0.458
        // Purple:      R=0.282 G=0.427 B=0.481
        // HoleGreen:   R=0.214 G=0.465 B=0.412
        // Green:       R=0.214 G=0.496 B=0.435

        public static double l1Range = 0.032;
        public static double m1Range = 0.040;
        public static double r1Range = 0.035;
        public static class P {
            public static double l1R = 0.230, l1G = 0.352, l1B = 0.363;
            public static double m1R = 0.287, m1G = 0.420, m1B = 0.460;
            public static double r1R = 0.275, r1G = 0.420, r1B = 0.470;
        }
        public static class G {
            public static double l1R = 0.196, l1G = 0.372, l1B = 0.342;
            public static double m1R = 0.210, m1G = 0.503, m1B = 0.395;
            public static double r1R = 0.214, r1G = 0.481, r1B = 0.424;
        }
    }
}