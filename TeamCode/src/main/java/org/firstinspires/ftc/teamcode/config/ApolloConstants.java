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
        RKICKER_DOWN = 0.02,
        LKICKER_UP = 0.24,
        MKICKER_UP = 0.24,
        RKICKER_UP = 0.26,
        HOOD_CLOSE = 0.2,
        HOOD_FAR = 0.85,
        SHOOTER_CLOSE = 0.75,
        SHOOTER_FAR = 0.9,
        SHOOTER_OFF = 0;

    public static DcMotorSimple.Direction
        frDir = DcMotorSimple.Direction.FORWARD,
        brDir = DcMotorSimple.Direction.FORWARD,
        flDir = DcMotorSimple.Direction.REVERSE,
        blDir = DcMotorSimple.Direction.REVERSE,
        intakeDir = DcMotorSimple.Direction.REVERSE,
        rShooterDir = DcMotorSimple.Direction.REVERSE,
        lShooterDir = DcMotorSimple.Direction.FORWARD;
}