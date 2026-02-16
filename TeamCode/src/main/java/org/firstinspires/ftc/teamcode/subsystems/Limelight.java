package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.Alliance.RED;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Pattern;

import java.util.List;

public class Limelight {

    Limelight3A l;

    private Pattern p = Pattern.NONE;
    private static final int shoot = 0, zone = 1;
    private int pipeline = shoot;

    public Limelight(HardwareMap hardwareMap) {
        l = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void switchToShoot() {
        if (pipeline != shoot)
            l.pipelineSwitch(shoot);
        l.setPollRateHz(20);
        l.start();
    }

    public Pattern motifDetection() {
        if (detectedID() == 21)
            p = Pattern.GPP21;
        else if (detectedID() == 22)
            p = Pattern.PGP22;
        else if (detectedID() == 23)
            p = Pattern.PPG23;
        return p;
    }

    public double detectedID() {
        switchToShoot();
        List<LLResultTypes.FiducialResult> r = l.getLatestResult().getFiducialResults();

        if (r.isEmpty()) return 0;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i: r) {
            if (i != null && i.getFiducialId() == 21)
                return 21;
            else if (i != null && i.getFiducialId() == 22)
                return 22;
            else if (i != null && i.getFiducialId() == 23)
                return 23;
        }

        return 0;
    }
}
