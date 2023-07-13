package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RRA", preselectTeleOp = "MECHANUM")
public class RoadRunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence first = drive.trajectorySequenceBuilder(new Pose2d(-37, 68, Math.toRadians(0.00)))
                .lineToConstantHeading(new Vector2d(-37, 44) )
                .build();
        drive.setPoseEstimate(first.start());

        waitForStart();
        if (opModeIsActive()) {
            while (!isStopRequested()) {
                drive.followTrajectorySequence(first);
                sleep(30000);
            }
        }
    }
}
