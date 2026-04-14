package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import dev.nextftc.core.commands.CommandManager;

public class CommandScheduler {
    LinearOpMode opmode;
    public CommandScheduler(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    public void initialize() {
        CommandManager.INSTANCE.preInit();
    }

    public void update() {
        if (opmode.opModeInInit()) CommandManager.INSTANCE.postWaitForStart();
        else if (opmode.opModeIsActive()) CommandManager.INSTANCE.postUpdate();
    }

    public void stop() {
        CommandManager.INSTANCE.postStop();
    }
}
