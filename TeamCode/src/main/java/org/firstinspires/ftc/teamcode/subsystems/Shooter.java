package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ApolloConstants.HOOD_MAX;
import static org.firstinspires.ftc.teamcode.config.ApolloConstants.HOOD_MIN;

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
    public static double kS = 0.385, kV = 0.00018, kP = 0.0025;
    public static double far_kS = 0.37, far_kV = 0.00027, far_kP = 0.0065;
    private boolean activated = true;
    public static double close = 1240;
    public static double far = 1725;
    public static double flipUp = 1;
    public static double flipDown = 0.7;
    public static double flipDownAutoOffset = 0.025;
    public static double hoodCorrection = 0.025;
    private boolean correctHood = false;
    private boolean up = false;
    public boolean isFar = false;
    public Shooter(HardwareMap hardwareMap) {
        l = hardwareMap.get(DcMotorEx.class, "lShooter");
        r = hardwareMap.get(DcMotorEx.class, "rShooter");
        f = hardwareMap.get(Servo.class, "hood");
        r.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return -l.getVelocity();
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
        isFar = true;
    }

    public void close() {
        setTarget(close);
        on();
        isFar = false;
    }

    public void setTarget(double velocity) {
        t = velocity;
    }

    @Override
    public void periodic() {

        if (activated) {
            double power;
            power = (kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS;
            if (isFar) power = (far_kV * getTarget()) + (far_kP * (getTarget() - getVelocity())) + far_kS;
            setPower(power);
        }

        double hoodCorrectFactor = (getTarget()-getVelocity()) * clamp((flipUp-flipDown)/(far-close), -0.5, 0.5);
        double hoodPos = flipUp;
        if (up)
            hoodPos = flipUp;
        else
            hoodPos = flipDown;
        if (correctHood)
            hoodPos -= hoodCorrectFactor * hoodCorrection;
        f.setPosition(clamp(hoodPos, HOOD_MIN, HOOD_MAX));
    }

    public void up() {
        up = true;
        f.setPosition(flipUp);
        correctHood = true;
    }

    public void down() {
        up = false;
        f.setPosition(flipDown);
        correctHood = true;
    }
    public void downAuto() {
        up = false;
        f.setPosition(flipDown+flipDownAutoOffset);
        correctHood = false;
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

