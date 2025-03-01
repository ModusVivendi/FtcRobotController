package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name="Parking_IntoDeep_LeftSide", group="IntoDeep")
public class IntoDeepParking_LeftSide extends LinearOpMode {

    private SampleMecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();

    // Field positions (adjust these based on actual field measurements)
    private static final Pose2d STARTING_POSE = new Pose2d(-10, -60, Math.toRadians(90)); // Adjust based on starting position
    private static final Pose2d Transit_1_POSITION = new Pose2d(-20, -58, Math.toRadians(90));  // First sample position
    private static final Pose2d PARKING_ZONE = new Pose2d(-20, -18, Math.toRadians(90)); // Human player zone position

    @Override
    public void runOpMode() {
        // Initialize hardware
        drive = new SampleMecanumDrive(hardwareMap);

        // Set initial pose
        drive.setPoseEstimate(STARTING_POSE);

        // Build trajectory to transition point 1
        Trajectory moveToTransit1 = drive.trajectoryBuilder(STARTING_POSE).lineToSplineHeading(Transit_1_POSITION)
                .build();

        // Build trajectory to parking zone
        Trajectory moveToParking = drive.trajectoryBuilder(Transit_1_POSITION).lineToSplineHeading(PARKING_ZONE)
                .build();

        // Wait for start
        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        // Execute small X move from the margin
        telemetry.addData("Status", "Moving to transit 1 position");
        telemetry.update();
        drive.followTrajectory(moveToTransit1);

        // Execute move to parking zone
        telemetry.addData("Status", "Moving to parking zone");
        telemetry.update();
        drive.followTrajectory(moveToParking);

        telemetry.addData("Status", "Complete!");
        telemetry.addData("Time", runtime.seconds());
        telemetry.update();
    }
}

