package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.LKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.MKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_DOWN;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.RKICKER_UP;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.blDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.brDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.flDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.frDir;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.intakeDir;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.ApolloConstants;
import org.firstinspires.ftc.teamcode.config.ApolloHardwareNames;
import org.firstinspires.ftc.teamcode.config.FieldPoses;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Drawing;

@TeleOp ()
public class blueVW extends LinearOpMode {
    Turret turret;
    Follower drive;
    Servo lKicker, mKicker, rKicker;
    Shooter shooter;
    DcMotor fl, bl, fr, br, intake;

    private double lKickerTarget = LKICKER_DOWN;
    private double mKickerTarget = MKICKER_DOWN;
    private double rKickerTarget = RKICKER_DOWN;
    private double intakePower = INTAKE_OFF;

    boolean targetPressed = false;
    double looptime = 0;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    boolean field = true;

    @Override
    public void runOpMode() {
        drive = Constants.createFollower(hardwareMap);
        turret = new Turret(hardwareMap);
        turret.off();
        shooter = new Shooter(hardwareMap);

        intake = hardwareMap.dcMotor.get("intake");
        lKicker = hardwareMap.servo.get(ApolloHardwareNames.lKicker);
        mKicker = hardwareMap.servo.get(ApolloHardwareNames.mKicker);
        rKicker = hardwareMap.servo.get(ApolloHardwareNames.rKicker);
        intake.setDirection(intakeDir);

        fl = hardwareMap.dcMotor.get(ApolloConstants.dt.fl);
        bl = hardwareMap.dcMotor.get(ApolloConstants.dt.bl);
        fr = hardwareMap.dcMotor.get(ApolloConstants.dt.fr);
        br = hardwareMap.dcMotor.get(ApolloConstants.dt.br);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(flDir);
        bl.setDirection(blDir);
        fr.setDirection(frDir);
        br.setDirection(brDir);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        while (opModeInInit()) {
            if (gamepad1.x) turret.resetTurret();
            telemetry.addData("Turret Angle",turret.getYaw());
            telemetry.addData("Get Turret", turret.getTurret());
            telemetry.update();

            intakePower = INTAKE_OFF;
            intake.setPower(intakePower);

        }

        waitForStart();
        boolean fieldToggle = false;

        shooter.close();
        shooter.down();
        while (opModeIsActive()) {
            turret.on();
            {
                Drawing.drawDebug(drive);
            }

            if(gamepad1.dpad_left)
                drive.setPose(new Pose(0,0,drive.getHeading()));

            if(gamepad1.options)
                drive.setPose(new Pose(drive.getPose().getX(), drive.getPose().getY() , 0));

            if (gamepad1.a && !fieldToggle)
                toggleField();
            fieldToggle = gamepad1.a;

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;


            if(field){
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double botHeading = drive.getPose().getHeading() + Math.PI/2;

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;
                fl.setPower(frontLeftPower);
                bl.setPower(backLeftPower);
                fr.setPower(frontRightPower);
                br.setPower(backRightPower);
            }
            else {

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;
                fl.setPower(frontLeftPower);
                bl.setPower(backLeftPower);
                fr.setPower(frontRightPower);
                br.setPower(backRightPower);
            }

            if (gamepad1.right_bumper) intakePower = INTAKE_IN;
            else if (gamepad1.left_bumper) intakePower = INTAKE_OUT;
            else intakePower = INTAKE_OFF;

            intake.setPower(intakePower);

            turret.face(FieldPoses.blueHoop, drive.getPose());

            if(gamepad1.dpad_down){
                shooter.up();
                shooter.far();
            }
            if (gamepad1.dpad_up) {
                shooter.down();
                shooter.close();
            }

            if (gamepad1.x) lKickerTarget = LKICKER_UP; else lKickerTarget = LKICKER_DOWN;
            if (gamepad1.y) mKickerTarget = MKICKER_UP; else mKickerTarget = MKICKER_DOWN;
            if (gamepad1.b) rKickerTarget = RKICKER_UP; else rKickerTarget = RKICKER_DOWN;

            lKicker.setPosition(lKickerTarget);
            mKicker.setPosition(mKickerTarget);
            rKicker.setPosition(rKickerTarget);

            turret.periodic();
            shooter.periodic();
            drive.updatePose();

            telemetry.addData("Robot Pose", drive.getPose());
            telemetry.addData("Turret Angle",turret.getYaw());
            telemetry.addData("Get Turret", turret.getTurret());
            telemetry.addData("Turret Target", turret.getTurretTarget());

            telemetry.addData("Shooter Current", shooter.getVelocity());
            telemetry.addData("Shooter  Target", shooter.getTarget());

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - looptime));
            looptime = loop;
            telemetry.update();
        }
    }

    private void toggleField() {
        if (field)
            field = false;
        else
            field = true;
    }
}
