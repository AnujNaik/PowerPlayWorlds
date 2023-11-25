package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Pooja;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/** Notes
 * Dropper Distance - 6-9 inches from center of junction (no more no less)
 */

@Disabled
@Config
@Autonomous
public class LeftAutoSafe extends LinearOpMode {
    Pooja pooja;
//    Thread lowerDepositCone5, lowerDepositCone4, lowerDepositCone3, lowerDepositCone2, lowerDepositCone1;
//    Thread collectCone5, collectCone4, collectCone3, collectCone2, collectCone1, transferCone5, transferCone4, transferCone3, transferCone2, transferCone1;

//    OpenCvWebcam webcam;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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

    @Override
    public void runOpMode() throws InterruptedException {
        pooja = new Pooja(hardwareMap);
//        lowerDepositCone5 = new LowerDepositCone();
//        collectCone5 = new CollectCone(5);
//        transferCone5 = new TransferCone();
//
//        lowerDepositCone4 = new LowerDepositCone();
//        collectCone4 = new CollectCone(4);
//        transferCone4 = new TransferCone();
//
//        lowerDepositCone3 = new LowerDepositCone();
//        collectCone3 = new CollectCone(3);
//        transferCone3 = new TransferCone();
//
//        lowerDepositCone2 = new LowerDepositCone();
//        collectCone2 = new CollectCone(2);
//        transferCone2 = new TransferCone();
//
//        lowerDepositCone1 = new LowerDepositCone();
//        collectCone1 = new CollectCone(1);
//        transferCone1 = new TransferCone();

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

        TrajectorySequence moveToCyclesPos = pooja.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(14, 3))
//                .splineTo(new Vector2d(27, -7), Math.toRadians(125))
//                .splineToSplineHeading(new Pose2d(32, -22, Math.toRadians(285)), Math.toRadians(150))
//                .waitSeconds(1)
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(40, -15), Math.toRadians(210))
//                .splineToSplineHeading(new Pose2d(-33, 11, Math.toRadians(0)), Math.toRadians(180))
                .build();


        waitForStart();

        if (isStopRequested()) return;


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
        pooja.followTrajectorySequence(moveToCyclesPos);
        sleep(3000);

//        pooja.followTrajectory(positionForNextJunction);
//        sleep(3000);
//
//        pooja.followTrajectory(moveToNextJunction);
//        sleep(3000);
//
//        pooja.followTrajectory(parkInZone);
//        sleep(3000);

//        pooja.raiseDepositSlides(telemetry);
//        pooja.lowerDepositSlides(telemetry);
//        pooja.extendCollectCone(5);
//        pooja.retractCollectCone();

//        collectCone5.start();
//        pooja.raiseDepositSlides(telemetry, "Stack Level 5");
//        lowerDepositCone5.start();
//        while (collectCone5.isAlive() && opModeIsActive()) {}
//        transferCone5.start();
//        while ((lowerDepositCone5.isAlive() || transferCone5.isAlive()) && opModeIsActive()) {}
//
//        sleep(1000);
//
//        collectCone4.start();
//        pooja.raiseDepositSlides(telemetry, "Stack Level 4");
//        lowerDepositCone4.start();
//        while (collectCone4.isAlive() && opModeIsActive()) {}
//        transferCone4.start();
//        while ((lowerDepositCone4.isAlive() || transferCone4.isAlive()) && opModeIsActive()) {}
//
//        sleep(1000);
//
//        collectCone3.start();
//        pooja.raiseDepositSlides(telemetry, "Stack Level 3");
//        lowerDepositCone3.start();
//        while (collectCone3.isAlive() && opModeIsActive()) {}
//        transferCone3.start();
//        while ((lowerDepositCone3.isAlive() || transferCone3.isAlive()) && opModeIsActive()) {}
//
//        sleep(1000);
//
//        collectCone2.start();
//        pooja.raiseDepositSlides(telemetry, "Stack Level 2");
//        lowerDepositCone2.start();
//        while (collectCone2.isAlive() && opModeIsActive()) {}
//        transferCone2.start();
//        while ((lowerDepositCone2.isAlive() || transferCone2.isAlive()) && opModeIsActive()) {}
//
//        sleep(1000);
//
//        collectCone1.start();
//        pooja.raiseDepositSlides(telemetry, "Stack Level 1");
//        lowerDepositCone1.start();
//        while (collectCone1.isAlive() && opModeIsActive()) {}
//        transferCone1.start();
//        while ((lowerDepositCone1.isAlive() || transferCone1.isAlive()) && opModeIsActive()) {}
//
//        sleep(10000);
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
                pooja.extendCollectCone(telemetry, this.stackLevel, "left");
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
                pooja.retractCollectCone(telemetry, 5, "left");
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            this.interrupt();
        }
    }
}