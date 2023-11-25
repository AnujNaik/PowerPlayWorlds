package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PID.PIDController;
import org.firstinspires.ftc.teamcode.drive.Pooja;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "BrakeMotorsTest")
public class BrakeMotorsTest extends LinearOpMode {
    Pooja pooja;
    public static double maxAllowableDisplacementError = 5.0;
    public static double maxAllowableHeadingError = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pooja drive = new Pooja(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        Pose2d initPos = drive.getPoseEstimate();

        while (!isStopRequested()) {
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Displacement", Math.sqrt(Math.pow(poseEstimate.getX(), 2) + Math.pow(poseEstimate.getY(), 2)));
            telemetry.update();

            if (poseEstimate.getX() - initPos.getX() > maxAllowableDisplacementError || poseEstimate.getY() - initPos.getY() > maxAllowableDisplacementError || poseEstimate.getHeading() - initPos.getHeading() > maxAllowableHeadingError) {
                Trajectory correction = drive.trajectoryBuilder(poseEstimate)
                        .lineToLinearHeading(initPos)
                        .build();

                drive.followTrajectory(correction);
                double displacement = Math.sqrt(Math.pow(poseEstimate.getX() - initPos.getX(), 2) + Math.pow(poseEstimate.getY() - initPos.getY(), 2));
                while (displacement > 0.1) {
                    poseEstimate = pooja.getPoseEstimate();
                    displacement = Math.sqrt(Math.pow(poseEstimate.getX() - initPos.getX(), 2) + Math.pow(poseEstimate.getY() - initPos.getY(), 2));
                }
            }
        }
    }
}