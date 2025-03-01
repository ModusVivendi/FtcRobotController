package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name="PushSamplesBySide_IntoDeep", group="IntoDeep")
public class AutoPushSideSamples extends LinearOpMode {
    private SampleMecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();
    private static final double xOffset =0.8;
    private static final double yOffset=0.97;

    // Field positions (adjust these based on actual field measurements)
    private static final Pose2d STARTING_POSE = new Pose2d(+10, -60, Math.toRadians(90)); // Adjust based on starting position
    private static final Pose2d Transit1_POSITION = new Pose2d(30*xOffset, -40, Math.toRadians(90));  // First sample position
    private static final Vector2d Transit1_POSITION_VECTOR = new Vector2d(30*xOffset, -40);  // First sample position
    private static final Pose2d SAMPLE1_POSITION_POSE = new Pose2d(40*xOffset, 0, Math.toRadians(90));  // First sample position
    private static final Vector2d SAMPLE1_POSITION = new Vector2d(40*xOffset, -10);  // First sample position
    private static final Vector2d SAMPLE2_POSITION = new Vector2d(50*xOffset, 0);  // Second sample position
    private static final Vector2d SAMPLE3_POSITION = new Vector2d(58*xOffset, 0);    // Third sample position
    private static final Vector2d PARKING1 = new Vector2d(40*xOffset, -56); // Human player zone position
    private static final Vector2d PARKING2 = new Vector2d(50*xOffset, -56); // Human player zone position
    private static final Vector2d PARKING3 = new Vector2d(58*xOffset, -56); // Human player zone position

    @Override
    public void runOpMode() {
        // Initialize hardware
        drive = new SampleMecanumDrive(hardwareMap);

        // Set initial pose
        drive.setPoseEstimate(STARTING_POSE);

        // Build trajectories
        Trajectory moveToTransit1 = drive.trajectoryBuilder(STARTING_POSE).lineTo(Transit1_POSITION_VECTOR)
                .build();

        // Build trajectories
        Trajectory moveToSample1 = drive.trajectoryBuilder(moveToTransit1.end()).splineToConstantHeading(SAMPLE1_POSITION, Math.toRadians(0))
                .build();


        // Build trajectories
        Trajectory pushToParking1 = drive.trajectoryBuilder(moveToSample1.end())
                .lineTo(PARKING1)
                .build();

        Trajectory moveToSample2 = drive.trajectoryBuilder(pushToParking1.end())
                .splineToConstantHeading(SAMPLE2_POSITION, Math.toRadians(90))
                .build();

        Trajectory pushToParking2 = drive.trajectoryBuilder(moveToSample2.end())
                .lineTo(PARKING2)
                .build();

        Trajectory moveToSample3 = drive.trajectoryBuilder(pushToParking2.end())
                .splineToConstantHeading(SAMPLE3_POSITION, Math.toRadians(90))
                .build();

        Trajectory pushToParking3 = drive.trajectoryBuilder(moveToSample3.end())
                .lineTo(PARKING3)
                .build();

//        Trajectory pushThirdSample = drive.trajectoryBuilder(pushToParking3.end())
//                .lineTo(HUMAN_PLAYER_ZONE)
//                .build();

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
        drive.followTrajectory(moveToSample1);

        telemetry.addData("Status", "Pushing first sample");
        telemetry.update();
        drive.followTrajectory(pushToParking1);

        telemetry.addData("Status", "Moving to second sample");
        telemetry.update();
        drive.followTrajectory(moveToSample2);

        telemetry.addData("Status", "Pushing second sample");
        telemetry.update();
        drive.followTrajectory(pushToParking2);

        telemetry.addData("Status", "Moving to third sample");
        telemetry.update();
        drive.followTrajectory(moveToSample3);

        telemetry.addData("Status", "Pushing third sample");
        telemetry.update();
        drive.followTrajectory(pushToParking3);

        telemetry.addData("Status", "Complete!");
        telemetry.addData("Time", runtime.seconds());
        telemetry.update();
    }
}
