package org.firstinspires.ftc.teamcode.Functions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawServos {

    private Servo servo;
    private boolean status1;
    public ClawServos(Servo _LS)
    {
        servo = _LS;
        status1 = true;
    }

    public void Open()
    {
        servo.setPosition(1);
    }
    public void Close() {
        servo.setPosition(0.5);
    }


}
