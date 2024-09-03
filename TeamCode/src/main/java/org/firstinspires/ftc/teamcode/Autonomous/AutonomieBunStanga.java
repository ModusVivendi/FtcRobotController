package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.EncoderMove;
import org.firstinspires.ftc.teamcode.Functions.NewEncoderMove;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AutonomieBun Stanga")
@Disabled
public class AutonomieBunStanga extends LinearOpMode {

    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private DcMotor leftMotor, leftMotorBack, rightMotor, rightMotorBack, armMotorLeft, armMotorRight;
    private Servo leftServo;
    private NewEncoderMove encoderMove;
    private ArmEncoder armEncoder;
    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam";
    private String currentPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        leftMotor = hardwareMap.dcMotor.get("FL");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        armMotorRight = hardwareMap.dcMotor.get("AMR");
        leftServo = hardwareMap.servo.get("LS");

        encoderMove = new NewEncoderMove(leftMotor,leftMotorBack,rightMotor,rightMotorBack);
        armEncoder = new ArmEncoder(armMotorLeft, armMotorRight);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        waitForStart();
        currentPosition = String.valueOf(sleeveDetection.getPosition());
        if(currentPosition=="RIGHT")
        {
            openServo(leftServo);
            sleep(1000);
            armEncoder.goTo(100,100);
            encoderMove.DriveTo(-150,-150,-150,-150,0.8,opModeIsActive());
            sleep(100);
            encoderMove.DriveTo(1250,-1250,-1250,1250,0.8,opModeIsActive());
            sleep(200);
            encoderMove.DriveTo(-1000,-1000,-1000,-1000,0.8,opModeIsActive());
            sleep(200);
            armEncoder.goTo(0,0);
        }
        else if(currentPosition=="CENTER")
        {
            openServo(leftServo);
            sleep(1000);
            armEncoder.goTo(100,100);
            encoderMove.DriveTo(-150,-150,-150,-150,0.8,opModeIsActive());
            sleep(100);
            encoderMove.DriveTo(1200,-1200,-1200,1200,0.8,opModeIsActive());
            sleep(200);
            encoderMove.DriveTo(-2000,-2000,-2000,-2000,0.8,opModeIsActive());
            sleep(200);
            encoderMove.DriveTo(1200,-1200,-1200,1200,0.8,opModeIsActive());
            sleep(200);
            armEncoder.goTo(0,0);
        }
        else if(currentPosition=="LEFT")
        {
            openServo(leftServo);
            sleep(1000);
            armEncoder.goTo(100,100);
            encoderMove.DriveTo(-150,-150,-150,-150,0.8,opModeIsActive());
            sleep(100);
            encoderMove.DriveTo(1250,-1250,-1250,1250,0.8,opModeIsActive());
            sleep(200);
            encoderMove.DriveTo(-1000,-1000,-1000,-1000,0.8,opModeIsActive());
            sleep(200);
            armEncoder.goTo(0,0);

        }

    }
    private void openServo(Servo _LS)
    {
        _LS.setPosition(1);

    }
    private void closeServo(Servo _LS)
    {
        _LS.setPosition(0);

    }
}
