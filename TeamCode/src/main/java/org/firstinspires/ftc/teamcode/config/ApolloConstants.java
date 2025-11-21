package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class ApolloConstants {

    public static double
        INTAKE_IN = 1,
        INTAKE_OUT = -1,
        INTAKE_OFF = 0.75;

    public static DcMotorSimple.Direction
        flDir = DcMotorSimple.Direction.FORWARD,
        blDir = DcMotorSimple.Direction.FORWARD,
        frDir = DcMotorSimple.Direction.REVERSE,
        brDir = DcMotorSimple.Direction.REVERSE,
        intakeDir = DcMotorSimple.Direction.FORWARD;
}