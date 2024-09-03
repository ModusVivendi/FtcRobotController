package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;


@Autonomous
public class RRAutonomousLeft extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    DcMotor rightRear;

    @Override
    public void runOpMode() {
        // Initialize your motors. Make sure the names match your configuration.
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        leftRear = hardwareMap.get(DcMotor.class, "BL");
        rightRear = hardwareMap.get(DcMotor.class, "BR");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Set the direction for motors if necessary
        // leftFront.setDirection(DcMotor.Direction.FORWARD);
        // rightFront.setDirection(DcMotor.Direction.REVERSE);
        // ...

        waitForStart();

        if (opModeIsActive()) {
            Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(30) // Adjust the distance to suit your needs
                    .build();

            drive.followTrajectory(trajectory);
        }
    }

    // You can remove camera-related methods and classes
}