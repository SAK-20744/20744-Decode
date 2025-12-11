package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class ApolloConstants {

    public static double
        INTAKE_IN = 1,
        INTAKE_OUT = -1,
        INTAKE_OFF = 0,
        LKICKER_DOWN = 0,
        MKICKER_DOWN = 0,
        RKICKER_DOWN = 0.1,
        LKICKER_UP = 0.27,
        MKICKER_UP = 0.31,
        RKICKER_UP = 0.39,
        HOOD_CLOSE = 0.2,
        HOOD_FAR = 0.9,
        SHOOTER_CLOSE = 0.65,
        SHOOTER_FAR = 0.8,
        SHOOTER_OFF = 0,
        VELOCITY_FAR = 400,
        VELOCITY_CLOSE = 275,
        tpl = 0.032,
        til = 0.021,
        tdl = 0.003,
        tps = 0.023,
        tis = 0.042,
        tds = 0.001,
        TURRET_THRESHOLD = 25;
    public static double
        KICKER_UPTIME = 0.3,
        KICKER_DOWNTIME = 0.1;
    public static int
        TURRET_LEFT = -50,
        TURRET_MIDDLE = 0,
        TURRET_RIGHT = 50;
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
}