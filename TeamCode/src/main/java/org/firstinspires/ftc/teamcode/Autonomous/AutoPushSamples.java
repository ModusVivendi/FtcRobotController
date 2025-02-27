package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name="PushSamples_IntoDeep", group="IntoDeep")
public class AutoPushSamples extends LinearOpMode {
    private SampleMecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();

    // Field positions (adjust these based on actual field measurements)
    private static final Pose2d STARTING_POSE = new Pose2d(+10, -60, Math.toRadians(90)); // Adjust based on starting position
    private static final Pose2d Transit1_POSITION = new Pose2d(30, -50, Math.toRadians(90));  // First sample position
    // private static final Vector2d Transit1_POSITION = new Vector2d(30, -10);  // First sample position
    private static final Pose2d Transit2_POSITION = new Pose2d(30, -10, Math.toRadians(90));  // First sample position

    private static final Vector2d SAMPLE1_POSITION = new Vector2d(35, -10);  // First sample position
    private static final Vector2d SAMPLE2_POSITION = new Vector2d(40, -24);  // Second sample position
    private static final Vector2d SAMPLE3_POSITION = new Vector2d(40, -15);    // Third sample position
    private static final Vector2d HUMAN_PLAYER_ZONE = new Vector2d(58, -12); // Human player zone position

    @Override
    public void runOpMode() {
        // Initialize hardware
        drive = new SampleMecanumDrive(hardwareMap);

        // Set initial pose
        drive.setPoseEstimate(STARTING_POSE);

        // Build trajectories
        Trajectory moveToTransit1 = drive.trajectoryBuilder(STARTING_POSE).lineToSplineHeading(Transit1_POSITION)
//                .lineTo(Transit1_POSITION)
                .build();

        // Build trajectories
        Trajectory moveToTransit2 = drive.trajectoryBuilder(Transit1_POSITION).lineToSplineHeading(Transit2_POSITION)
                .build();


        // Build trajectories
        Trajectory moveToFirstSample = drive.trajectoryBuilder(moveToTransit2.end())
                .lineTo(SAMPLE1_POSITION)
                .build();

        Trajectory pushFirstSample = drive.trajectoryBuilder(moveToFirstSample.end())
                .lineTo(HUMAN_PLAYER_ZONE)
                .build();

        Trajectory returnFromFirst = drive.trajectoryBuilder(pushFirstSample.end())
                .lineTo(SAMPLE2_POSITION)
                .build();

        Trajectory pushSecondSample = drive.trajectoryBuilder(returnFromFirst.end())
                .lineTo(HUMAN_PLAYER_ZONE)
                .build();

        Trajectory returnFromSecond = drive.trajectoryBuilder(pushSecondSample.end())
                .lineTo(SAMPLE3_POSITION)
                .build();

        Trajectory pushThirdSample = drive.trajectoryBuilder(returnFromSecond.end())
                .lineTo(HUMAN_PLAYER_ZONE)
                .build();

        // Wait for start
        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        // Execute sample pushing sequence
        telemetry.addData("Status", "Moving to first sample");
        telemetry.update();
        drive.followTrajectory(moveToTransit1);

        // Execute sample pushing sequence
        telemetry.addData("Status", "Moving to first sample");
        telemetry.update();
        drive.followTrajectory(moveToFirstSample);

        telemetry.addData("Status", "Pushing first sample");
        telemetry.update();
        drive.followTrajectory(pushFirstSample);

        telemetry.addData("Status", "Moving to second sample");
        telemetry.update();
        drive.followTrajectory(returnFromFirst);

//        telemetry.addData("Status", "Pushing second sample");
//        telemetry.update();
//        drive.followTrajectory(pushSecondSample);
//
//        telemetry.addData("Status", "Moving to third sample");
//        telemetry.update();
//        drive.followTrajectory(returnFromSecond);
//
//        telemetry.addData("Status", "Pushing third sample");
//        telemetry.update();
//        drive.followTrajectory(pushThirdSample);

        telemetry.addData("Status", "Complete!");
        telemetry.addData("Time", runtime.seconds());
        telemetry.update();
    }
}
