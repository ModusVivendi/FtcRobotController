package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.CRServo;

public class ArmServos {
    private CRServo servoCrLeft, servoCrRight;
    private boolean statusCr, statusInv;

    public ArmServos(CRServo L1S, CRServo L2S) {
        servoCrLeft = L1S;
        servoCrRight = L2S;
        statusCr = false;
        statusInv = false;
    }

    /**
     * This method starts the servo.
     */
    public void Start()
    {
        servoCrLeft.setPower(-1);
        servoCrRight.setPower(1);
        statusCr=true;
        statusInv=false;
    }
    /**
     * This method starts the servo in the opposite direction.
     */
    public void StartInv()
    {
        servoCrLeft.setPower(1);
        servoCrRight.setPower(-1);
        statusCr=false;
        statusInv=true;
    }
    /**
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     * This method stops the servo.
     */
    public void Stop()
    {
        servoCrLeft.setPower(0);
        servoCrRight.setPower(0);
        statusCr=false;
        statusInv = false;
    }


    /**
     * This method starts/stops the vacuum depending on the value of status(variable).
     */
    public void Switch()
    {
        if(statusCr)
        {
            Stop();
        }
        else {
            Start();
        }
    }
    public void SwitchInv()
    {
        if(statusInv)
        {
            Stop();
        }
        else
        {
            Start();
        }
    }


    double currentWaitTime =0;
    double currentTimeStamp =0;

    /**
     * This method checks if x seconds have passed, and if that's true => .
     */

    public void SwitchAndWaitContinuous(double x, double currentRuntime)
    {
        if(currentRuntime == 0 || currentTimeStamp + currentWaitTime <= currentRuntime) {
            Switch();
            currentTimeStamp = currentRuntime;
            currentWaitTime = x;
        }
    }
    public void SwitchAndWaitContinuousInv(double x, double currentRuntime)
    {
        if(currentRuntime == 0 || currentTimeStamp + currentWaitTime <= currentRuntime) {
            SwitchInv();
            currentTimeStamp = currentRuntime;
            currentWaitTime = x;
        }
    }
}
