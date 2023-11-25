package org.firstinspires.ftc.teamcode.autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PID.PIDController;
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
public class LeftAutoMid<XPOS> extends LinearOpMode {
   Pooja pooja;
   Thread lowerDepositCone5, lowerDepositCone4, lowerDepositCone3, lowerDepositCone2, lowerDepositCone1;
   Thread collectCone5, collectCone4, collectCone3, collectCone2, collectCone1, transferCone5, transferCone4, transferCone3, transferCone2, transferCone1;

   OpenCvWebcam webcam;
   AprilTagDetectionPipeline aprilTagDetectionPipeline;

   double fx = 578.272;
   double fy = 578.272;
   double cx = 402.145;
   double cy = 221.506;

   double tagsize = 0.166;

   int LEFT = 1;
   int MIDDLE = 2;
   int RIGHT = 3;

   int parkingZone = 2;

   public static double XPOS = 37.5;
   public static double YPOS = 2;
   public static double ANGLE = 95;
   public static double DISTANCE = 11;

   @Override
   public void runOpMode() throws InterruptedException {
       pooja = new Pooja(hardwareMap);
//       lowerDepositCone5 = new LowerDepositCone();
//       collectCone5 = new CollectCone(5);
//       transferCone5 = new TransferCone();
//
//       lowerDepositCone4 = new LowerDepositCone();
//       collectCone4 = new CollectCone(4);
//       transferCone4 = new TransferCone();
//
//       lowerDepositCone3 = new LowerDepositCone();
//       collectCone3 = new CollectCone(3);
//       transferCone3 = new TransferCone();
//
//       lowerDepositCone2 = new LowerDepositCone();
//       collectCone2 = new CollectCone(2);
//       transferCone2 = new TransferCone();
//
//       lowerDepositCone1 = new LowerDepositCone();
//       collectCone1 = new CollectCone(1);
//       transferCone1 = new TransferCone();

    //    collectCone5
    //    depositCone6

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
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(15, 2.7), Math.toRadians(185))
//                .splineToSplineHeading(new Pose2d(42.5, 6.5, Math.toRadians(75)), Math.toRadians(185))
               .lineToLinearHeading(new Pose2d(XPOS, YPOS, Math.toRadians(ANGLE)))
               .build();

       Trajectory moveToIntakePos = pooja.trajectoryBuilder(moveToCyclesPos.end())
               .lineToLinearHeading(new Pose2d(48.5, 3, Math.toRadians(ANGLE)))
//               .strafeRight(DISTANCE)
               .build();

       Trajectory moveToOuttakePos = pooja.trajectoryBuilder(moveToIntakePos.end())
               .lineToLinearHeading(new Pose2d(XPOS, YPOS, Math.toRadians(ANGLE)))
//               .strafeLeft(DISTANCE)
               .build();

       TrajectorySequence parkZone1 = pooja.trajectorySequenceBuilder(moveToCyclesPos.end())
                       .setReversed(false)
                       .splineToSplineHeading(new Pose2d(47, 10.5, Math.toRadians(270)), Math.toRadians(90))
                       .splineTo(new Vector2d(47, 31), Math.toRadians(90))
                       .build();

       TrajectorySequence parkZone3 = pooja.trajectorySequenceBuilder(moveToCyclesPos.end())
               .setReversed(true)
               .lineToLinearHeading(new Pose2d(27.5, 6, Math.toRadians(180)))
               .strafeLeft(24)
               .build();


       waitForStart();

       if (isStopRequested()) return;


       ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
       for(AprilTagDetection tag: currentDetections) {
           if(tag.id == LEFT) {
               parkingZone = 1;
           }
           else if(tag.id == MIDDLE) {
               parkingZone = 2;
           }
           else if(tag.id == RIGHT) {
               parkingZone = 3;
           }
       }
       telemetry.addData("Zone", parkingZone);
       telemetry.update();

       if (isStopRequested()) return;

       // Get in cycle position
       pooja.followTrajectory(moveToCyclesPos);
       sleep(1000);

       pooja.followTrajectory(moveToIntakePos);
       pooja.extendCollectCone(telemetry, 5, "AUTO");
       pooja.retractCollectCone(telemetry, 5, "AUTO");
       ANGLE = ANGLE - 1;

       sleep(1000);

       pooja.followTrajectory(moveToOuttakePos);
       pooja.raiseDepositSlides(telemetry, "mid");
       pooja.lowerDepositSlides(telemetry);
       ANGLE = ANGLE - 1;

       sleep(1000);


       pooja.followTrajectory(moveToIntakePos);
       pooja.extendCollectCone(telemetry, 4, "AUTO");
       pooja.retractCollectCone(telemetry, 4, "AUTO");
       ANGLE = ANGLE - 1;

       sleep(1000);

       pooja.followTrajectory(moveToOuttakePos);
       pooja.raiseDepositSlides(telemetry, "mid");
       pooja.lowerDepositSlides(telemetry);
       ANGLE = ANGLE - 1;

       sleep(1000);


       pooja.followTrajectory(moveToIntakePos);
       pooja.extendCollectCone(telemetry, 3, "AUTO");
       pooja.retractCollectCone(telemetry, 3, "AUTO");
       ANGLE = ANGLE - 1;

       sleep(1000);

       pooja.followTrajectory(moveToOuttakePos);
       pooja.raiseDepositSlides(telemetry, "mid");
       pooja.lowerDepositSlides(telemetry);
       ANGLE = ANGLE - 1;

       sleep(1000);


       pooja.followTrajectory(moveToIntakePos);
       pooja.extendCollectCone(telemetry, 3, "AUTO");
       pooja.retractCollectCone(telemetry, 3, "AUTO");
       ANGLE = ANGLE - 1;

       sleep(1000);

       pooja.followTrajectory(moveToOuttakePos);
       pooja.raiseDepositSlides(telemetry, "mid");
       pooja.lowerDepositSlides(telemetry);
       ANGLE = ANGLE - 1;

       sleep(1000);


       pooja.followTrajectory(moveToIntakePos);
       pooja.extendCollectCone(telemetry, 2, "AUTO");
       pooja.retractCollectCone(telemetry, 2, "AUTO");
       ANGLE = ANGLE - 1;

       sleep(1000);

       pooja.followTrajectory(moveToOuttakePos);
       pooja.raiseDepositSlides(telemetry, "mid");
       pooja.lowerDepositSlides(telemetry);
       ANGLE = ANGLE - 1;

       sleep(1000);


       pooja.followTrajectory(moveToIntakePos);
       pooja.extendCollectCone(telemetry, 1, "AUTO");
       pooja.retractCollectCone(telemetry, 1, "AUTO");
       ANGLE = ANGLE - 1;

       sleep(1000);

       pooja.followTrajectory(moveToOuttakePos);
       pooja.raiseDepositSlides(telemetry, "high");
       pooja.lowerDepositSlides(telemetry);
       ANGLE = ANGLE - 1;

       sleep(1000);

   }

//    private class LowerDepositCone extends Thread {
//        public LowerDepositCone() {}

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

//    private class CollectCone extends Thread {
//        int stackLevel;

//        public CollectCone(int stackLevel) {
//            this.stackLevel = stackLevel;
//        }

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

//    private class TransferCone extends Thread {
//        public TransferCone() {}

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
}