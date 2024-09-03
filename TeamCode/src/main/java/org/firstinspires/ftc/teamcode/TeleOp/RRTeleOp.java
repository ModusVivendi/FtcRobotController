package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.ClawServos;
import org.firstinspires.ftc.teamcode.Functions.GamepadCalc;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.PIDController;
import org.firstinspires.ftc.teamcode.Functions.PlaneLuncher;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.TopServos;
import org.firstinspires.ftc.teamcode.Functions.Vacuum;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;

@Disabled
@TeleOp(name="RRTeleOp1", group = "GAME")
public class RRTeleOp extends LinearOpMode {
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack, vacumMotor;
    private DcMotor armMotorLeft, armMotorRight;
    private Servo clawServo, rightServo, leftServo,planeServo;
    private ArmEncoder controller;
    private Move move;
    private Vacuum vacum;
//    private Rotate rotate;
    private ClawServos clawServos;
    private PlaneLuncher plane;
//    private ArmEncoder armEncoder;
    private TopServos topServos;
//    private Servo servo;
    private boolean status1;
    int armLevel = 0;
    double integralSum = 0;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 2;
    double movement;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    String armCurrentDirection = "up";
    private ElapsedTime runtime = new ElapsedTime();
    final double END_GAME = 90.0;
    final double FIFTEEN_SECONDS = 105.0;
    final double FIVE_SECONDS = 115.0;
    boolean secondFifteen = false;
    boolean secondEnd = false;
    boolean secondFive = false;
    int count = 400;
    boolean goDown = true;
    Gamepad.RumbleEffect customRumbleEffectFive, customRumbleEffectFifteen, customRumbleEffectEnd;
    double speed = .7;

    GamepadCalc gamepadCalc;

    @Override
    public void runOpMode() throws InterruptedException {

        int tickAdjustment = 100;

        customRumbleEffectFive = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .build();

        customRumbleEffectFifteen = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 600)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 600)  //  Rumble left motor 100% for 250 mSec
                .build();

        customRumbleEffectEnd = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 700)  //  Rumble right motor 100% for 500 mSec
                .build();

        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        vacumMotor = hardwareMap.dcMotor.get("AS");
        armMotorLeft = hardwareMap.dcMotor.get("SL");
        armMotorRight = hardwareMap.dcMotor.get("SR");
        leftServo = hardwareMap.servo.get("ASL");
        rightServo = hardwareMap.servo.get("ASR");
        clawServo = hardwareMap.servo.get("CS");
        planeServo = hardwareMap.servo.get("PL");
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        vacum = new Vacuum(vacumMotor);
        clawServos = new ClawServos(clawServo);
        controller = new ArmEncoder(armMotorLeft, armMotorRight);
        topServos = new TopServos(leftServo, rightServo);
        plane = new PlaneLuncher(planeServo);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        gamepadCalc = new GamepadCalc(this);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        runtime.reset(); // Start game timer.

        if (isStopRequested()) return;
        topServos.Close();
        clawServos.Close();
        while(opModeIsActive() && !isStopRequested()) {

            //Watch the runtime timer, and run the custom rumble when we hit half-time.
            //Make sure we only signal once by setting "secondHalf" flag to prevent further rumbles.
            if ((runtime.seconds() > FIFTEEN_SECONDS) && !secondFifteen)  {
                gamepad1.runRumbleEffect(customRumbleEffectFifteen);
                gamepad2.runRumbleEffect(customRumbleEffectFifteen);
                secondFifteen =true;
            }
            else if ((runtime.seconds() > END_GAME) && !secondEnd)  {
                gamepad1.runRumbleEffect(customRumbleEffectEnd);
                gamepad2.runRumbleEffect(customRumbleEffectEnd);
                secondEnd =true;
            }
            else if ((runtime.seconds() > FIVE_SECONDS) && !secondFive)  {
                gamepad1.runRumbleEffect(customRumbleEffectFive);
                gamepad2.runRumbleEffect(customRumbleEffectFive);
                secondFive =true;
            }


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

            if(gamepad2.a){
                vacum.StartInvers();
            }
            if(gamepad2.y){
                vacum.Start();
            }
            if (gamepad2.left_bumper){
                clawServos.Open();
                vacum.Stop();
            }
            if(gamepad2.right_bumper){
                clawServos.Close();
            }

            if(gamepad1.touchpad){
                plane.Open();
            }
            if (gamepad2.dpad_left){
                clawServos.Open();
                topServos.Open();
                armCurrentDirection = "up";
                controller.goTo(2000, 2000);

            }
            if (gamepad2.dpad_down){
                clawServos.Close();
                topServos.Close();
                armCurrentDirection = "down";
                controller.goTo(0, 0);
                //
            }
            if(gamepad2.dpad_up) // Level 2
            {
                clawServos.Open();
                topServos.Open();
                armCurrentDirection = "up";
                controller.goTo(4700, 4700
                );
            }
            if(gamepad2.dpad_right) // Level 2
            {
                clawServos.Open();
                topServos.Open();
                armCurrentDirection = "up";
                controller.goTo(3500, 3500);
            }
            if(gamepad2.left_trigger > 0.1) { // Adjust based on your deadzone preference
                // Decrease target position based on left trigger pressure
                armMotorLeft.setPower(-gamepad2.right_trigger / 10);
                armMotorRight.setPower(-gamepad2.right_trigger / 10);
            }

            if(gamepad2.right_trigger > 0) { // Adjust based on your deadzone preference
                // Increase target position based on right trigger pressure
                armMotorLeft.setPower(gamepad2.right_trigger / 10);
                armMotorRight.setPower(gamepad2.right_trigger / 10);

                telemetry.addData("Going up", gamepad1.right_trigger * tickAdjustment);
                telemetry.update();
            }
            if (gamepad2.right_stick_button){
                armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("Encoder Reseted", "true");
                telemetry.update();
            }

            if(gamepad2.left_stick_y<0)
            {
                armMotorRight.setPower(gamepad2.left_stick_y/3);
                armMotorLeft.setPower(gamepad2.left_stick_y/3);
            }
            if(gamepad2.left_stick_y>   0)
            {
                armMotorRight.setPower(gamepad2.left_stick_y/3);
                armMotorLeft.setPower(gamepad2.left_stick_y/3);
            }

        }

    }


    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }
    private void openServo(Servo _LS)
    {
        _LS.setPosition(1);
    }
    private void closeServo(Servo _LS)
    {
        _LS.setPosition(0);
    }
    private void servosUp(Servo topLeftServo)
    {
        topLeftServo.setPosition(0);
    }
    private void servosDown(Servo topLeftServo)
    {
        topLeftServo.setPosition(1);
    }


}