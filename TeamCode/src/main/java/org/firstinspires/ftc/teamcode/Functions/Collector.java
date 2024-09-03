package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.CRServo;

public class Collector {
    private CRServo servoCrLeft;
    private CRServo servoCrRight;
    private boolean status, statusCr,statusInv;

    // Start da start la servoCr
    // Start da start la servo
    /**
     * This method initialises the motors.
     * @param : servo
     */
    public Collector(CRServo _servoCr, CRServo _servoCrRight)
    {
        servoCrLeft = _servoCr;
        servoCrRight = _servoCrRight;
        statusCr=false;
        statusInv=false;

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
            StartInv();
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
    /**
     * This is how to read/use status variable.
     * @return : (boolean) This returns value of status variable.
     */
    public boolean CheckStatus(){
        return status;
    }

    double currentWaitTimeshort =0;
    double currentWaitTimelong=0;

    public void SwitchAndWait(double waitTimeshort, double waitTimelong, double currentRuntime) {
        double currentWaitAux=0;
        // acum verificam daca e pornit servoul sau nu
        if(statusCr) {
            // daca e pornit asteptam mai mult
            currentWaitAux=waitTimelong;
        }
        else{
            // daca nu mai putin
            currentWaitAux=waitTimeshort;
        }
        if (currentRuntime ==  0 || currentTimeStamp + currentWaitAux <= currentRuntime) {
            Switch();
            currentTimeStamp = currentRuntime;
            currentWaitTimeshort = waitTimeshort;
            currentWaitTimelong = waitTimelong;
        }


    }

}
