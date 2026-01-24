package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.config.ApolloConstants;

@Config
@Configurable

public class Shooter extends SubsystemBase {
    private Servo f;
    private DcMotorEx l, r;

    private double t = 0;
    public static double kS = 0.5, kV = 0.00022, kP = 0.003;
    private boolean activated = true;
    public static double close = 1450;
    public static double far = 1950;
    public static double flipUp = ApolloConstants.HOOD_FAR;
    public static double flipDown = ApolloConstants.HOOD_CLOSE;
    public static double hoodCorrection = 1;
    private boolean hoodCorrect = true;
    private boolean up = false;

    public Shooter(HardwareMap hardwareMap) {
        l = hardwareMap.get(DcMotorEx.class, "lShooter");
        r = hardwareMap.get(DcMotorEx.class, "rShooter");
        f = hardwareMap.get(Servo.class, "hood");
        l.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return r.getVelocity();
    }

    public void setPower(double p) {
        l.setPower(p);
        r.setPower(p);
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public boolean isActivated() {
        return activated;
    }

    public void hoodCorrect() {

    }

    public void far() {
        setTarget(far);
        on();
    }

    public void close() {
        setTarget(close);
        on();
    }

    public void setTarget(double velocity) {
        t = velocity;
    }

    @Override
    public void periodic() {
        if (activated)
            setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);

        double hoodCorrectFactor = (getTarget()-getVelocity()) * clamp((flipUp-flipDown)/(far-close), -0.5, 0.5);
        double hoodPos = flipUp;
        if (up)
            hoodPos = flipUp;
        else
            hoodPos = flipDown;
        if (hoodCorrect)
            hoodPos -= hoodCorrectFactor * hoodCorrection;
        f.setPosition(clamp(hoodPos, 0.2, 0.9));
    }

    public void up() {
        up = true;
        f.setPosition(flipUp);
    }

    public void down() {
        up = false;
        f.setPosition(flipDown);
    }

    public void flip() {
        if (f.getPosition() == flipDown)
            up();
        else
            down();
    }

    public boolean atTarget() {
        return Math.abs((getTarget()- getVelocity())) < ApolloConstants.shooterVelError;
    }

    public void forDistance(double distance) {
        //setTarget((6.13992 * distance) + 858.51272);

//        setTarget((0.00180088*Math.pow(distance, 2))+(4.14265*distance)+948.97358);
    }

    public boolean atUp() {
        return f.getPosition() == flipUp;
    }

    public double clamp(double val, double min, double max) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }
}

