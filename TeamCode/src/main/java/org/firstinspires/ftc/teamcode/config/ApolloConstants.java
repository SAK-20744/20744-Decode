package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class ApolloConstants {

    public static double
        INTAKE_IN = 1,
        INTAKE_OUT = -1,
        INTAKE_OFF = 0,
        LKICKER_DOWN = 0.15,
        MKICKER_DOWN = 0.15,
        RKICKER_DOWN = 0.15,
        LKICKER_UP = 0.6,
        MKICKER_UP = 0.6,
        RKICKER_UP = 0.6,
        HOOD_CLOSE = 0.4,
        HOOD_FAR = 0.6,
        SHOOTER_CLOSE = 0.75,
        SHOOTER_FAR = 0.9,
        SHOOTER_OFF = 0;

    public static DcMotorSimple.Direction
        flDir = DcMotorSimple.Direction.FORWARD,
        blDir = DcMotorSimple.Direction.FORWARD,
        frDir = DcMotorSimple.Direction.REVERSE,
        brDir = DcMotorSimple.Direction.REVERSE,
        intakeDir = DcMotorSimple.Direction.FORWARD,
        rShooterDir = DcMotorSimple.Direction.REVERSE,
        lShooterDir = DcMotorSimple.Direction.FORWARD;
}