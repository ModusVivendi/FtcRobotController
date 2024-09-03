package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.Servo;

public class TopServos {

    private Servo topServoLeft, topServoRight;
    private boolean status1;
    public TopServos(Servo _TSL, Servo _TSR)
    {
        topServoLeft = _TSL;
        topServoRight = _TSR;
        status1 = false;
    }
    public void Open() {
        // Set servos to 1/10 open position
        topServoLeft.setPosition(0.45);
        topServoRight.setPosition(0.55);

        status1 = true;
    }

    public void Close() {
        // Set servos to 9/10 closed position
        topServoLeft.setPosition(0.90);
        topServoRight.setPosition(0.10);
        status1 = false;
    }


    public void Switch(){
        if(status1){
            Close();
        }
        else{
            Open();
        }
    }
    double currentWaitTime =0;
    double currentTimeStamp =0;
    public void SwitchAndWait(double x, double currentRuntime){
        if(currentWaitTime ==0|| currentTimeStamp + currentWaitTime <=currentRuntime){
            Switch();
            currentTimeStamp =currentRuntime;
            currentWaitTime =x;
        }
    }

}
