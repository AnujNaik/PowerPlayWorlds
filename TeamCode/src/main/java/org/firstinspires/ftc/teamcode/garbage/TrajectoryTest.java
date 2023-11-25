package org.firstinspires.ftc.teamcode.garbage;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PID.PIDController;
import org.firstinspires.ftc.teamcode.drive.Pooja;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Vector;

/** Notes
 * Dropper Distance - 6-9 inches from center of junction (no more no less)
 */

@Disabled
@Config
@Autonomous
public class TrajectoryTest extends LinearOpMode {
    Pooja pooja;
    Thread lowerDepositCone5, lowerDepositCone4, lowerDepositCone3, lowerDepositCone2, lowerDepositCone1;
    Thread collectCone5, collectCone4, collectCone3, collectCone2, collectCone1, transferCone5, transferCone4, transferCone3, transferCone2, transferCone1;

    OpenCvWebcam webcam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static double XPOS = 58.4, YPOS = 0.0, ANGLE = 114;

//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    double tagsize = 0.166;
//
//    int LEFT = 1;
//    int MIDDLE = 2;
//    int RIGHT = 3;
//
//    int parkingZone = 2;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        pooja = new Pooja(hardwareMap);
//        PIDController collectorPID = new PIDController(0.00482, 0.000012, 0.00001);
//        lowerDepositCone5 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.LowerDepositCone();
//        collectCone5 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.CollectCone(5);
//        transferCone5 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.TransferCone();
//
//        lowerDepositCone4 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.LowerDepositCone();
//        collectCone4 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.CollectCone(4);
//        transferCone4 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.TransferCone();

//        lowerDepositCone3 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.LowerDepositCone();
//        collectCone3 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.CollectCone(3);
//        transferCone3 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.TransferCone();
//
//        lowerDepositCone2 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.LowerDepositCone();
//        collectCone2 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.CollectCone(2);
//        transferCone2 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.TransferCone();
//
//        lowerDepositCone1 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.LowerDepositCone();
//        collectCone1 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.CollectCone(1);
//        transferCone1 = new org.firstinspires.ftc.teamcode.autonomous.LeftAutoSharedHigh.TransferCone();

//        collectCone5
//        depositCone6

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "backWebcam"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        webcam.setPipeline(aprilTagDetectionPipeline);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(960,720, OpenCvCameraRotation.UPSIDE_DOWN);
//            }
//
//            @Override
//            public void onError(int errorCode) { }
//        });

//        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        pooja.setPoseEstimate(startPose);

        Trajectory moveToCyclesPos = pooja.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(XPOS, 0))
                .build();
//
//        Trajectory positionForNextJunction = pooja.trajectoryBuilder(moveToCyclesPos.end())
//                .splineToConstantHeading(new Vector2d(54, 4), Math.toRadians(55))
//                .splineToLinearHeading(new Pose2d(45, -5.0, Math.toRadians(0)), Math.toRadians(75))
//                .build();
//
//        Trajectory moveToNextJunction = pooja.trajectoryBuilder(positionForNextJunction.end())
//                .splineToLinearHeading(new Pose2d(45, -35, Math.toRadians(0)), Math.toRadians(105))
//                .build();
//
//        Trajectory parkInZone = pooja.trajectoryBuilder(moveToNextJunction.end())
//                .lineToLinearHeading(new Pose2d(58.5, 7.0, Math.toRadians(111)))
//                .build();


        waitForStart();

        if (isStopRequested()) return;

//        pooja.followTrajectory(moveToCyclesPos);

//        while (opModeIsActive()) {

//        pooja.extendCollectorSlidesEnhanced(telemetry, 600);
//        sleep(2000);


//        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//        for(AprilTagDetection tag: currentDetections) {
//            if(tag.id == LEFT) {
//                parkingZone = 1;
//            }
//            else if(tag.id == MIDDLE) {
//                parkingZone = 2;
//            }
//            else if(tag.id == RIGHT) {
//                parkingZone = 3;
//            }
//        }
//        telemetry.addData("Zone", parkingZone);
//        telemetry.update();

//        if (isStopRequested()) return;
//
//        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//        for(AprilTagDetection tag: currentDetections) {
//            if(tag.id == LEFT) {
//                parkingZone = 1;
//            }
//            else if(tag.id == MIDDLE) {
//                parkingZone = 2;
//            }
//            else if(tag.id == RIGHT) {
//                parkingZone = 3;
//            }
//        }
//
//        telemetry.addData("Zone", parkingZone);
//        telemetry.update();

        // Get in cycle position
        pooja.followTrajectory(moveToCyclesPos);
        pooja.turn(Math.toRadians(114));
        sleep(10000);
//
//        pooja.followTrajectory(positionForNextJunction);
//        sleep(3000);
//
//        pooja.followTrajectory(moveToNextJunction);
//        sleep(3000);
//
//        pooja.followTrajectory(parkInZone);
//        sleep(3000);

//        pooja.brakeMotorPowers();
//        pooja.extendCollectCone(telemetry, 5);
//        sleep(2000);
//        pooja.retractCollectCone(telemetry);
//        sleep(500);
////
//        pooja.extendCollectCone(telemetry, 5);
//        sleep(2000);
//        pooja.retractCollectCone(telemetry);
//        sleep(500);
//
//        pooja.extendCollectCone(telemetry, 5);
//        sleep(2000);
//        pooja.retractCollectCone(telemetry);
//        sleep(500);
//
//        pooja.extendCollectCone(telemetry, 5);
//        sleep(2000);
//        pooja.retractCollectCone(telemetry);
//        sleep(500);
//
//        pooja.extendCollectCone(telemetry, 5);
//        sleep(2000);
//        pooja.retractCollectCone(telemetry);
//        sleep(500);
//
//        pooja.extendCollectCone(telemetry, 5);
//        sleep(2000);
//        pooja.retractCollectCone(telemetry);
//        sleep(500);
//
//
//        pooja.extendCollectCone(telemetry, 5);
//        sleep(2000);
//        pooja.retractCollectCone(telemetry);
//        sleep(500);
//
//
//        pooja.extendCollectCone(telemetry, 5);
//        sleep(2000);
//        pooja.retractCollectCone(telemetry);
//        sleep(500);
//
//
//        pooja.extendCollectCone(telemetry, 5);
//        sleep(2000);
//        pooja.retractCollectCone(telemetry);
//        sleep(500);
//
//
//        pooja.extendCollectCone(telemetry, 5);
//        sleep(2000);
//        pooja.retractCollectCone(telemetry);
//        sleep(500);
//
//
//        pooja.extendCollectCone(telemetry, 5);
//        sleep(2000);
//        pooja.retractCollectCone(telemetry);
//        sleep(500);

//        sleep(5000);

//        collectCone5.start();
//        pooja.raiseDepositSlides(telemetry, "high");
//        lowerDepositCone5.start();
//        while (collectCone5.isAlive() && opModeIsActive()) {}
//        transferCone5.start();
//        while ((lowerDepositCone4.isAlive() || transferCone5.isAlive()) && opModeIsActive()) {}
//
//        sleep(200);
//
//
//        collectCone4.start();
//        pooja.raiseDepositSlides(telemetry, "high");
//        lowerDepositCone4.start();
//        while (collectCone4.isAlive() && opModeIsActive()) {}
//        transferCone4.start();
//        while ((lowerDepositCone4.isAlive() || transferCone4.isAlive()) && opModeIsActive()) {}
//
//        sleep(200);
//
//        collectCone3.start();
//        pooja.raiseDepositSlides(telemetry, "high");
//        lowerDepositCone3.start();
//        while (collectCone3.isAlive() && opModeIsActive()) {}
//        transferCone3.start();
//        while ((lowerDepositCone3.isAlive() || transferCone3.isAlive()) && opModeIsActive()) {}
//
//        sleep(200);
//
//        collectCone2.start();
//        pooja.raiseDepositSlides(telemetry, "high");
//        lowerDepositCone2.start();
//        while (collectCone2.isAlive() && opModeIsActive()) {}
//        transferCone2.start();
//        while ((lowerDepositCone2.isAlive() || transferCone2.isAlive()) && opModeIsActive()) {}
//
//        sleep(200);
//
//        collectCone1.start();
//        pooja.raiseDepositSlides(telemetry, "high");
//        lowerDepositCone1.start();
//        while (collectCone1.isAlive() && opModeIsActive()) {}
//        transferCone1.start();
//        while ((lowerDepositCone1.isAlive() || transferCone1.isAlive()) && opModeIsActive()) {}
//
//            sleep(10000);

//            break;
//
//        }
    }

    private class LowerDepositCone extends Thread {
        public LowerDepositCone() {}

        @Override
        public void run() {
            try {
                pooja.lowerDepositSlides(telemetry);
            } catch (Exception e) {
                e.printStackTrace();
            }
            this.interrupt();
        }
    }

    private class CollectCone extends Thread {
        int stackLevel;

        public CollectCone(int stackLevel) {
            this.stackLevel = stackLevel;
        }

        @Override
        public void run() {
            try {
                pooja.extendCollectCone(telemetry, this.stackLevel, "neutral");
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            this.interrupt();
        }
    }

    private class TransferCone extends Thread {
        public TransferCone() {}

        @Override
        public void run() {
            try {
                pooja.retractCollectCone(telemetry, 5, "neutral");
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            this.interrupt();
        }
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
