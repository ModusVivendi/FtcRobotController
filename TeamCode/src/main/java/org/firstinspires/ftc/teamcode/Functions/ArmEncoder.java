package org.firstinspires.ftc.teamcode.Functions;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
public class ArmEncoder {

    private final DcMotor armMotorLeft, armMotorRight;
    private int armLeftPos, armRightPos;
    double integralSum = 0;
    public static double kp = 0.01;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;
    private double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    public void Init()
    {
        armMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotorLeft.setDirection(DcMotor.Direction.REVERSE);

        armLeftPos = 0;
        armRightPos = 0;
    }

    public ArmEncoder(DcMotor _AML, DcMotor _AMR)
    {
        armMotorLeft= _AML;
        armMotorRight= _AMR;
        Init();
    }

    public double PIDControl(double reference, double state)
    {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kp) + (derivative * kd) + (integralSum * ki) + (reference * kf);
        return output;

    }

    public void goTo(int armLeftTarget, int armRightTarget)
    {
        armLeftPos = 0;
        armRightPos = 0;

        armLeftPos+=armLeftTarget;
        armRightPos+=armRightTarget;

        armMotorLeft.setTargetPosition(armLeftPos);
        armMotorRight.setTargetPosition(armRightPos);

        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double powerLeft = PIDControl(armLeftTarget, armMotorLeft.getCurrentPosition());
        double powerRight = PIDControl(armRightTarget, armMotorRight.getCurrentPosition());

        armMotorLeft.setPower(1);
        armMotorRight.setPower(1);
    }




}