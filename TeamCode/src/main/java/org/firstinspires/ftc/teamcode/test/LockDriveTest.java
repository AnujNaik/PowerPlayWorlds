package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Pooja;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Vector;

@Disabled
@Config
@Autonomous
public class LockDriveTest extends LinearOpMode {
    Pooja pooja;
    Trajectory goToLockPos;

    @Override
    public void runOpMode() throws InterruptedException {
        pooja = new Pooja(hardwareMap);
        Pose2d lockPos = new Pose2d(10, 10, Math.toRadians(90));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        pooja.setPoseEstimate(startPose);

        waitForStart();

        goToLockPos = pooja.trajectoryBuilder(pooja.getPoseEstimate())
                .lineTo(new Vector2d(lockPos.getX(), lockPos.getY()))
                .addDisplacementMarker(() -> {
                    sleep(1500);
                    if (pooja.getPoseEstimate().getX() - lockPos.getX() > 1 || pooja.getPoseEstimate().getY() - lockPos.getY() > 1) {
                        pooja.followTrajectory(goToLockPos);
                    }
                })
                .build();

        pooja.followTrajectory(goToLockPos);

//        while (!isStopRequested()) {
//            double distanceFromLockPos = Math.sqrt((Math.pow(pooja.getPoseEstimate().getX() - lockPos.getX(), 2)) + (Math.pow(pooja.getPoseEstimate().getY() - lockPos.getY(), 2)));
//            if (distanceFromLockPos > 5) {
//                Trajectory goToLockPos = pooja.trajectoryBuilder(pooja.getPoseEstimate())
//                        .lineToLinearHeading(new Pose2d(lockPos.getX(), lockPos.getY(), lockPos.getHeading()))
//                        .build();
//
//                pooja.followTrajectory(goToLockPos);
//            }
//        }

    }
}