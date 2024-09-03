package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Move2 {

    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private int currentDirection;

    private void Init()
    {
        try{
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            currentDirection = 0;
        }
        catch(NullPointerException e)
        {

        }
    }

    /**
     * This is the constructor of the class
     * @param _LM , this is the left motor
     * @param _RM
     * @param _LMB
     * @param _RMB
     */
    public Move2(DcMotor _LM, DcMotor _RM, DcMotor _LMB, DcMotor _RMB)
    {
        leftMotor = _LM;
        rightMotor = _RM;
        leftMotorBack = _LMB;
        rightMotorBack = _RMB;
        Init();
    }

    /**
     * 1 - fata , 2 - spate, 3 - stanga, 4 - dreapta
     * @param direction
     * @param power
     */
    public void MoveRaw(int direction, double power)
    {
        try {
            switch(direction) {
                case 1:
                    leftMotor.setPower(-power);
                    leftMotorBack.setPower(-power);
                    rightMotor.setPower(power);
                    rightMotorBack.setPower(power);
                    break;
                case 2:
                    leftMotor.setPower(power);
                    leftMotorBack.setPower(power);
                    rightMotor.setPower(-power);
                    rightMotorBack.setPower(-power);
                    break;
                case 3:
                    leftMotor.setPower(power);
                    leftMotorBack.setPower(-power);
                    rightMotor.setPower(power);
                    rightMotorBack.setPower(-power);
                    break;
                case 4:
                    leftMotor.setPower(-power);
                    leftMotorBack.setPower(power);
                    rightMotor.setPower(-power);
                    rightMotorBack.setPower(power);
                    break;
            }
        }
        catch(NullPointerException e)
        {

        }
    }

    public void Rotate(int direction)
    {
        switch (direction)
        {
            //left
            case 1:
                leftMotor.setPower(1);
                leftMotorBack.setPower(1);
                rightMotorBack.setPower(1);
                rightMotor.setPower(1);
                break;
            case 2:
                leftMotorBack.setPower(-1);
                leftMotor.setPower(-1);
                rightMotor.setPower(-1);
                rightMotorBack.setPower(-1);
                break;
        }
    }
    public void MoveOne( int direction)
    {
        try
        {
            switch(direction)
            {
                case 1:
                    leftMotor.setPower(0.8);
                    break;
                case 2:
                    rightMotor.setPower(0.8);
                    break;
                case 3:
                    leftMotorBack.setPower(0.8);
                    break;
                case 4:
                    rightMotorBack.setPower(0.8);
                    break;
            }
        }
        catch(NullPointerException e)
        {

        }
    }

    public void MoveStop()
    {
        leftMotor.setPower(0);
        leftMotorBack.setPower(0);
        rightMotor.setPower(0);
        rightMotorBack.setPower(0);
    }

    public void MoveFull(int direction)
    {
        MoveRaw(direction, 0.8);
    }

}
