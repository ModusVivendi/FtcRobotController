package org.firstinspires.ftc.teamcode.Orig.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.GamepadCalc;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;

@TeleOp(name="RRTeleOpORIG", group = "ORIG")
public class RRTeleOp extends LinearOpMode {
    //Declare motors
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private DcMotor armMotorLeft, armMotorRight;

    //Declare servos
    private Servo leftBall, rightBall;

    //Other declarations
    double integralSum = 0;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 2;
    double movement;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private ElapsedTime runtime = new ElapsedTime();


    //declare classes
    private ArmEncoder controller;
    GamepadCalc gamepadCalc;



    public void runOpMode() throws InterruptedException {
        int tickAdjustment = 100;

        //init motors
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        armMotorLeft = hardwareMap.dcMotor.get("SL");
        armMotorRight = hardwareMap.dcMotor.get("SR");

        //init servos
        //leftBall = hardwareMap.servo.get("LB");
        //rightBall = hardwareMap.servo.get("RB");

        controller = new ArmEncoder(armMotorLeft, armMotorRight);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        gamepadCalc = new GamepadCalc(this);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        runtime.reset(); // Start game timer.

        if (isStopRequested()) return;
        while(opModeIsActive() && !isStopRequested()) {

            gamepadCalc.calculate();
            movement = gamepadCalc.getGamepad1().left_trigger - gamepadCalc.getGamepad1().right_trigger;

            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(
                    -
                            gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );


            drive.update();
            drive.update();

            if(gamepad1.right_bumper)
            {
                drive.setPoseEstimate(PoseStorage.currentPose);
                telemetry.addData("Heading reseted to: ", PoseStorage.currentPose);
                telemetry.update();
            }
            if(gamepad1.dpad_up)
            {
                controller.goTo(4700, 4700);
            }
            if(gamepad1.dpad_down) {
                controller.goTo(0,0);
            }

        }
    }

    //PID Control
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }

}
