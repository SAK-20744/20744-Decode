package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.OFFSET;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.TURRET_MIDDLE;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.TURRET_THRESHOLD;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.VELOCITY_CLOSE;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.VELOCITY_FAR;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.blDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.brDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.dt;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.flDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.frDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.intakeDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.tdl;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.tds;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.til;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.tis;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.tpl;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.tps;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.turretDir;
import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.Alliance.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.config.ApolloHardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.List;

@TeleOp
public class NewRobotTele extends OpMode {

    private double looptime = 0;
    DcMotor fl, bl, fr, br, intake;

    public void init() {

        fl = hardwareMap.dcMotor.get(dt.fl);
        bl = hardwareMap.dcMotor.get(dt.bl);
        fr = hardwareMap.dcMotor.get(dt.fr);
        br = hardwareMap.dcMotor.get(dt.br);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(flDir);
        bl.setDirection(blDir);
        fr.setDirection(frDir);
        br.setDirection(brDir);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    public void init_loop(){
        telemetry.update();
    }

    @Override
    public void loop() {
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - looptime));
        looptime = loop;
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() { super.stop(); }

}