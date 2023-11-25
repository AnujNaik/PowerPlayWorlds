package org.firstinspires.ftc.teamcode.autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Pooja;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

/** Notes
 * Dropper Distance - 6-9 inches from center of junction (no more no less)
 */

@Config
@Autonomous
public class RightAutoSharedHighPreloaded extends LinearOpMode {
    Pooja pooja;
    Thread lowerDepositCone5, lowerDepositCone4, lowerDepositCone3, lowerDepositCone2, lowerDepositCone1;
    Thread collectCone5, collectCone4, collectCone3, collectCone2, collectCone1, transferCone5, transferCone4, transferCone3, transferCone2, transferCone1;

    OpenCvWebcam webcam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static double XPOS = 58.2, YPOS = 2.5, ANGLE = 263; // 112.7, 58.2, 5.0
    public static double XPOS_park = 49, YPOS_park = 1, ANGLE_park = 190;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    int parkingZone = 2;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        pooja = new Pooja(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "backWebcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(960,720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) { }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        pooja.setPoseEstimate(startPose);

        Trajectory moveToCyclesPos = pooja.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(XPOS, YPOS, Math.toRadians(ANGLE)))
                .build();

        TrajectorySequence parkInZone1 = pooja.trajectorySequenceBuilder(moveToCyclesPos.end())
                .lineToLinearHeading(new Pose2d(XPOS_park, YPOS_park, Math.toRadians(ANGLE_park)))
                .strafeLeft(24)
                .build();

        Trajectory parkInZone2 = pooja.trajectoryBuilder(moveToCyclesPos.end())
                .lineToLinearHeading(new Pose2d(XPOS_park, YPOS_park, Math.toRadians(ANGLE_park)))
                .build();

        TrajectorySequence parkInZone3 = pooja.trajectorySequenceBuilder(moveToCyclesPos.end())
                .lineToLinearHeading(new Pose2d(XPOS_park, YPOS_park, Math.toRadians(ANGLE_park)))
                .strafeRight(24)
                .build();


        telemetry.addData("CD Reading ", pooja.getCollectorSensorReading());
        telemetry.addData("Deposit starting position", pooja.retrieveDropperEncoderCount());
        telemetry.update();

        waitForStart();

        pooja.FBToRest();

        if (isStopRequested()) return;


        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        for(AprilTagDetection tag: currentDetections) {
            if(tag.id == LEFT) {
                parkingZone = 1;
            }
            else if (tag.id == MIDDLE) {
                parkingZone = 2;
            }
            else if (tag.id == RIGHT) {
                parkingZone = 3;
            }
        }

        telemetry.addData("Parking Zone", parkingZone);
        telemetry.update();

        // Get in cycle position
        pooja.followTrajectory(moveToCyclesPos);
        sleep(400);

        pooja.raiseDepositSlides(telemetry, "high");
        pooja.lowerDepositSlides(telemetry);

        if (parkingZone == 1) {
            pooja.followTrajectorySequence(parkInZone1);
        }
        else if (parkingZone == 3) {
            pooja.followTrajectorySequence(parkInZone3);
        }
        else {
            pooja.followTrajectory(parkInZone2);
        }

        pooja.setDepositSlidesPower(-1.0);
        sleep(200);
        pooja.setDepositSlidesPower(0.0);
    }
}

/** Collector Test
 * pooja.raiseDepositSlides(telemetry, "high");
 *             pooja.lowerDepositSlides(telemetry);
 *             sleep(1000);
 *             pooja.raiseDepositSlides(telemetry, "high");
 *             pooja.lowerDepositSlides(telemetry);
 *             sleep(1000);
 *             pooja.raiseDepositSlides(telemetry, "high");
 *             pooja.lowerDepositSlides(telemetry);
 *             sleep(1000);
 *             pooja.raiseDepositSlides(telemetry, "high");
 *             pooja.lowerDepositSlides(telemetry);
 *             sleep(1000);
 *             pooja.raiseDepositSlides(telemetry, "high");
 *             pooja.lowerDepositSlides(telemetry);
 *             sleep(1000);
 */

// Dropper Test
//pooja.raiseDepositSlides(telemetry, "high");
//        sleep(200);
//        pooja.lowerDepositSlides(telemetry);
//        sleep(2000);
//        pooja.raiseDepositSlides(telemetry, "high");
//        sleep(200);
//        pooja.lowerDepositSlides(telemetry);
//        sleep(2000);
//        pooja.raiseDepositSlides(telemetry, "high");
//        sleep(200);
//        pooja.lowerDepositSlides(telemetry);
//        sleep(2000);
//        pooja.raiseDepositSlides(telemetry, "high");
//        sleep(200);
//        pooja.lowerDepositSlides(telemetry);
//        sleep(2000);
//        pooja.raiseDepositSlides(telemetry, "high");
//        sleep(200);
//        pooja.lowerDepositSlides(telemetry);
//        sleep(2000);
//        pooja.raiseDepositSlides(telemetry, "high");
//        sleep(200);
//        pooja.lowerDepositSlides(telemetry);
//        sleep(2000);
//        pooja.raiseDepositSlides(telemetry, "high");
//        sleep(200);
//        pooja.lowerDepositSlides(telemetry);
//        sleep(2000);