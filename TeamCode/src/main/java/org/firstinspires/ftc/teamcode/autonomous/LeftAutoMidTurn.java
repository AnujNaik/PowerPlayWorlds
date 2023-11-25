//package org.firstinspires.ftc.teamcode.autonomous;
//
//import android.annotation.SuppressLint;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.PID.PIDController;
//import org.firstinspires.ftc.teamcode.drive.Pooja;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//import java.util.ArrayList;
//
///** Notes
// * Dropper Distance - 6-9 inches from center of junction (no more no less)
// */
//
//@Config
//@Autonomous
//public class LeftAutoMidTurn extends LinearOpMode {
//    Pooja pooja;
//    Thread lowerDepositCone5, lowerDepositCone4, lowerDepositCone3, lowerDepositCone2, lowerDepositCone1;
//    Thread collectCone5, collectCone4, collectCone3, collectCone2, collectCone1, transferCone5, transferCone4, transferCone3, transferCone2, transferCone1;
//    Thread brake;
//
//    OpenCvWebcam webcam;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    public static double XPOS = 58, YPOS = 10, ANGLE = 40;
//    public static double XPOS_park = 49, YPOS_park = 6, ANGLE_park = 190;
//
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
//
//    @SuppressLint("SuspiciousIndentation")
//    @Override
//    public void runOpMode() throws InterruptedException {
//        pooja = new Pooja(hardwareMap);
//        PIDController collectorPID = new PIDController(0.00482, 0.000012, 0.00001);
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
//
//        brake = new Brake();
//
//
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
//
//        FtcDashboard.getInstance().startCameraStream(webcam, 0);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
//        pooja.setPoseEstimate(startPose);
//
//        Trajectory moveToMidIntial= pooja.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(XPOS, YPOS, Math.toRadians(ANGLE)))
//                .build();
//
//        TrajectorySequence parkInZone1 = pooja.trajectorySequenceBuilder(moveToCyclesPos.end())
//                .lineToLinearHeading(new Pose2d(XPOS_park, YPOS_park, Math.toRadians(ANGLE_park)))
//                .strafeRight(24)
//                .build();
//
//        Trajectory parkInZone2 = pooja.trajectoryBuilder(moveToCyclesPos.end())
//                .lineToLinearHeading(new Pose2d(XPOS_park, YPOS_park, Math.toRadians(ANGLE_park)))
//                .build();
//
//        TrajectorySequence parkInZone3 = pooja.trajectorySequenceBuilder(moveToCyclesPos.end())
//                .lineToLinearHeading(new Pose2d(XPOS_park, YPOS_park, Math.toRadians(ANGLE_park)))
//                .strafeLeft(24)
//                .build();
//
//
//        telemetry.addData("CD Reading ", pooja.getCollectorSensorReading());
//        telemetry.addData("Deposit starting position", pooja.retrieveDropperEncoderCount());
//        telemetry.update();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//
//        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//        for(AprilTagDetection tag: currentDetections) {
//            if(tag.id == LEFT) {
//                parkingZone = 1;
//            }
//            else if (tag.id == MIDDLE) {
//                parkingZone = 2;
//            }
//            else if (tag.id == RIGHT) {
//                parkingZone = 3;
//            }
//        }
//
//        telemetry.addData("Parking Zone", parkingZone);
//        telemetry.update();
//
//        // Get in cycle position
//        pooja.followTrajectory(moveToMidIntial);
//        sleep(400);
////        brake.start();
//
//        pooja.brakeMotorPowers();
//        pooja.FBToIntake(5);
//        sleep(300);
//        pooja.raiseDepositSlides(telemetry, "high");
//        pooja.lowerDepositSlides(telemetry);
//
//        pooja.turn(Math.toRadians(50));
//
//        pooja.extendCollectCone(telemetry, 5);
//        pooja.retractCollectCone(telemetry, 5);
//
//        pooja.turn(Math.toRadians(-50));
//
//        pooja.raiseDepositSlides(telemetry, "high");
//        pooja.lowerDepositSlides(telemetry);
////        sleep(1000);
//        pooja.turn(Math.toRadians(50));
//
//        pooja.extendCollectCone(telemetry, 4);
//        pooja.retractCollectCone(telemetry, 4);
//
//        pooja.turn(Math.toRadians(-50));
//
//        pooja.raiseDepositSlides(telemetry, "high");
//        pooja.lowerDepositSlides(telemetry);
////        sleep(1000);
//        pooja.turn(Math.toRadians(50));
//
//        pooja.extendCollectCone(telemetry, 3);
//        pooja.retractCollectCone(telemetry, 3);
//
//        pooja.turn(Math.toRadians(-50));
//
//        pooja.raiseDepositSlides(telemetry, "high");
//        pooja.lowerDepositSlides(telemetry);
////        sleep(1000);
//        pooja.turn(Math.toRadians(50));
//
//        pooja.extendCollectCone(telemetry, 2);
//        pooja.retractCollectCone(telemetry, 2);
//
//        pooja.turn(Math.toRadians(-50));
//
//        pooja.raiseDepositSlides(telemetry, "high");
//        pooja.lowerDepositSlides(telemetry);
////        sleep(1000);
//        pooja.turn(Math.toRadians(50));
//
//        pooja.extendCollectCone(telemetry, 1);
//        pooja.retractCollectCone(telemetry, 1);
//        pooja.turn(Math.toRadians(-50));
//
//        pooja.raiseDepositSlides(telemetry, "high");
//        pooja.lowerDepositSlides(telemetry);
//        pooja.FBToTransfer();
//
//        if (parkingZone == 1) {
//            pooja.followTrajectorySequence(parkInZone1);
//        }
//        else if (parkingZone == 3) {
//            pooja.followTrajectorySequence(parkInZone3);
//        }
//        else {
//            pooja.followTrajectory(parkInZone2);
//        }
//
//        pooja.setDepositSlidesPower(-1.0);
//        sleep(200);
//        pooja.setDepositSlidesPower(0.0);
//    }
//
//    private class LowerDepositCone extends Thread {
//        public LowerDepositCone() {}
//
//        @Override
//        public void run() {
//            try {
//                pooja.lowerDepositSlides(telemetry);
//            } catch (Exception e) {
//                e.printStackTrace();
//            }
//            this.interrupt();
//        }
//    }
//
//    private class CollectCone extends Thread {
//        int stackLevel;
//
//        public CollectCone(int stackLevel) {
//            this.stackLevel = stackLevel;
//        }
//
//        @Override
//        public void run() {
//            try {
//                pooja.extendCollectCone(telemetry, this.stackLevel);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            this.interrupt();
//        }
//    }
//
//    private class TransferCone extends Thread {
//        public TransferCone() {}
//
//        @Override
//        public void run() {
//            try {
//                pooja.retractCollectCone(telemetry, 5);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            this.interrupt();
//        }
//    }
//
//    private class Brake extends Thread {
//        Pose2d initialPosition = pooja.getPoseEstimate();
//
//        public Brake() {}
//
//        @Override
//        public void run() {
//            while (true) {
//                if (pooja.getPoseEstimate().getX() - initialPosition.getX() > pooja.brakeTolerance || pooja.getPoseEstimate().getY() - initialPosition.getY() > pooja.brakeTolerance || pooja.getPoseEstimate().getHeading() - initialPosition.getHeading() > pooja.brakeTolerance) {
//                    pooja.correctPosition(initialPosition);
//                }
//            }
//        }
//    }
//}
//
///** Collector Test
// * pooja.raiseDepositSlides(telemetry, "high");
// *             pooja.lowerDepositSlides(telemetry);
// *             sleep(1000);
// *             pooja.raiseDepositSlides(telemetry, "high");
// *             pooja.lowerDepositSlides(telemetry);
// *             sleep(1000);
// *             pooja.raiseDepositSlides(telemetry, "high");
// *             pooja.lowerDepositSlides(telemetry);
// *             sleep(1000);
// *             pooja.raiseDepositSlides(telemetry, "high");
// *             pooja.lowerDepositSlides(telemetry);
// *             sleep(1000);
// *             pooja.raiseDepositSlides(telemetry, "high");
// *             pooja.lowerDepositSlides(telemetry);
// *             sleep(1000);
// */
//
//// Dropper Test
////pooja.raiseDepositSlides(telemetry, "high");
////        sleep(200);
////        pooja.lowerDepositSlides(telemetry);
////        sleep(2000);
////        pooja.raiseDepositSlides(telemetry, "high");
////        sleep(200);
////        pooja.lowerDepositSlides(telemetry);
////        sleep(2000);
////        pooja.raiseDepositSlides(telemetry, "high");
////        sleep(200);
////        pooja.lowerDepositSlides(telemetry);
////        sleep(2000);
////        pooja.raiseDepositSlides(telemetry, "high");
////        sleep(200);
////        pooja.lowerDepositSlides(telemetry);
////        sleep(2000);
////        pooja.raiseDepositSlides(telemetry, "high");
////        sleep(200);
////        pooja.lowerDepositSlides(telemetry);
////        sleep(2000);
////        pooja.raiseDepositSlides(telemetry, "high");
////        sleep(200);
////        pooja.lowerDepositSlides(telemetry);
////        sleep(2000);
////        pooja.raiseDepositSlides(telemetry, "high");
////        sleep(200);
////        pooja.lowerDepositSlides(telemetry);
////        sleep(2000);