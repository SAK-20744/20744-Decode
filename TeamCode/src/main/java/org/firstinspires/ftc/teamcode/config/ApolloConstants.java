package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Pattern;
import org.opencv.core.Scalar;

@Config
public class ApolloConstants {

    public static double autoTurret = 115;
    public static double autoTurret2 = 115;
    public static double autoTurret3 = 115;

    public static double TurretScaling = 0.18;

    public static double eject = 0.4;

    public static double blueautoTurret = -116;
    public static double blueautoTurret2 = -116;
    public static double blueautoTurret3 = -116;

    public static double intakeMovementSpeed = 0.35;

    public static double shooterVelError = 40;

    public static Pattern motif = Pattern.GPP21;

    public static double
        INTAKE_IN = 1,
        INTAKE_OUT = -1,
        INTAKE_OFF = 0,
        LKICKER_DOWN = 0.025,
        MKICKER_DOWN = 0.01,
        RKICKER_DOWN = 0.025,
        LKICKER_UP = 0.295,
        MKICKER_UP = 0.295,
        RKICKER_UP = 0.295,
        HOOD_MIN = 0.34,
        HOOD_MAX = 1,
//        HOOD_CLOSE = 0.22,
//        HOOD_FAR = 0.7025,
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
        TILT_RETRACT = 0.58,
        TILT_EXTEND = 0.34;

    public static double shooterkS = 0.135, shooterkV = 0.000525, shooterkP = 0.02;

    public static double REDLIGHT = 0.277;
    public static double PURPLELIGHT = 0.722;
    public static double UP_TIME = 2.0;

    public static double CAM_FWD_IN  = 5.0;   // camera offset forward from turret center
    public static double CAM_LEFT_IN = 0.5;   // camera offset left from turret center

    public static double VISION_MAX_JUMP_IN = 0; // inches
    public static double VISION_ALPHA = 0;       // blend factor

    public static double VELOCITY_CORRECTION = 0.5;

//    // OWENS CODE. ONLY FOR FULL SHOOTER TEST
//    public static double
//        KICKER_UPTIME = 0.5,
//        KICKER_DOWNTIME = 0.3;

    public static double
        KUP = 200,
        KDOWN = 80;
    public static double
            KUP_SLOW = 300,
            KDOWN_SLOW = 125;
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
    @Config
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

        public static double l1RRange = 0.05, l1GRange = 0, l1BRange = 0;
        public static double m1RRange = 0.05;
        public static double r1RRange = 0.05;

        public static class P {
//            public static double l1R = 0.230, l1G = 0.352, l1B = 0.363;
//            public static double m1R = 0.287, m1G = 0.420, m1B = 0.460;
//            public static double r1R = 0.275, r1G = 0.420, r1B = 0.470;

            public static double l1R = 0, l1G = 0, l1B = 0;
            public static double m1R = 0, m1G = 0, m1B = 0;
            public static double r1R = 0, r1G = 0, r1B = 0;
        }

        public static class G {
            public static double l1R = 0.240, l1G = 0.375, l1B = 0.385;
            public static double m1R = 0.295, m1G = 0.700, m1B = 0.605;
            public static double r1R = 0.265, r1G = 0.595, r1B = 0.545;
        }
    }
    public static class colorRanges {
        public Scalar lGMin = new Scalar(0.190, 0.325, 0.335);
        public Scalar lGMax = new Scalar(0.290, 0.425, 0.435);
        public Scalar mGMin = new Scalar(0.245, 0.650, 0.555);
        public Scalar mGMax = new Scalar(0.345, 0.750, 0.655);
        public Scalar rGMin = new Scalar(0.215, 0.545, 0.495);
        public Scalar rGMax = new Scalar(0.315, 0.645, 0.595);

        public Scalar l2GMin = new Scalar(0.190, 0.325, 0.335);
        public Scalar l2GMax = new Scalar(0.290, 0.425, 0.435);
        public Scalar m2GMin = new Scalar(0.245, 0.650, 0.555);
        public Scalar m2GMax = new Scalar(0.345, 0.750, 0.655);
        public Scalar r2GMin = new Scalar(0.215, 0.545, 0.495);
        public Scalar r2GMax = new Scalar(0.315, 0.645, 0.595);
    }

}