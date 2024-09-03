package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadCalc {
    private OpMode opmode;

    Gamepad gamepad1;

    public GamepadCalc(OpMode opmode)
    {
        this.opmode = opmode;
    }

    public Gamepad getGamepad1()
    {
        return gamepad1;
    }

    public void calculate ()
    {
        gamepad1 = opmode.gamepad1;

    }
}
