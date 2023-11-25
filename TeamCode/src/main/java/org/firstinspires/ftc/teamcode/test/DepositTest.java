package org.firstinspires.ftc.teamcode.test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PID.PIDController;
import org.firstinspires.ftc.teamcode.drive.Pooja;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

/** Notes
 * Dropper Distance - 6-9 inches from center of junction (no more no less)
 */

@Disabled
@Config
@Autonomous
public class DepositTest extends LinearOpMode {
    Pooja pooja;
    Thread lowerDepositCone5, lowerDepositCone4, lowerDepositCone3, lowerDepositCone2, lowerDepositCone1;
    Thread collectCone5, collectCone4, collectCone3, collectCone2, collectCone1, transferCone5, transferCone4, transferCone3, transferCone2, transferCone1;


    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        pooja = new Pooja(hardwareMap);
//        PIDController collectorPID = new PIDController(0.00482, 0.000012, 0.00001);
        lowerDepositCone5 = new LowerDepositCone();
        collectCone5 = new CollectCone(5);
        transferCone5 = new TransferCone();

        lowerDepositCone4 = new LowerDepositCone();
        collectCone4 = new CollectCone(4);
        transferCone4 = new TransferCone();

        lowerDepositCone3 = new LowerDepositCone();
        collectCone3 = new CollectCone(3);
        transferCone3 = new TransferCone();

        lowerDepositCone2 = new LowerDepositCone();
        collectCone2 = new CollectCone(2);
        transferCone2 = new TransferCone();

        lowerDepositCone1 = new LowerDepositCone();
        collectCone1 = new CollectCone(1);
        transferCone1 = new TransferCone();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;


        pooja.brakeMotorPowers();
        pooja.engageConeStabilizer();
        pooja.extendCollectCone(telemetry, 5, "neutral");
        sleep(400);
        pooja.retractCollectCone(telemetry, 5, "neutral");
        sleep(100);
        pooja.raiseDepositSlides(telemetry, "high");
        sleep(200);
        pooja.lowerDepositSlides(telemetry);
        sleep(2000);

        pooja.extendCollectCone(telemetry, 4, "neutral");
        sleep(400);
        pooja.retractCollectCone(telemetry, 5, "neutral");
        sleep(100);
        pooja.raiseDepositSlides(telemetry, "high");
        sleep(200);
        pooja.lowerDepositSlides(telemetry);
        sleep(2000);

        pooja.extendCollectCone(telemetry, 3, "neutral");
        sleep(400);
        pooja.retractCollectCone(telemetry, 5, "neutral");
        sleep(100);
        pooja.raiseDepositSlides(telemetry, "high");
        sleep(200);
        pooja.lowerDepositSlides(telemetry);
        sleep(2000);

        pooja.extendCollectCone(telemetry, 2, "neutral");
        sleep(400);
        pooja.retractCollectCone(telemetry, 5, "neutral");
        sleep(100);
        pooja.raiseDepositSlides(telemetry, "high");
        sleep(200);
        pooja.lowerDepositSlides(telemetry);
        sleep(2000);

        pooja.extendCollectCone(telemetry, 1, "neutral");
        sleep(400);
        pooja.retractCollectCone(telemetry, 5, "neutral");
        sleep(100);
        pooja.raiseDepositSlides(telemetry, "high");
        sleep(200);
        pooja.lowerDepositSlides(telemetry);
        sleep(2000);

        sleep(5000);

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

//        sleep(2000);
//
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
//        pooja.raiseDepositSlides(telemetry, "high");
//        sleep(200);
//        pooja.lowerDepositSlides(telemetry);
//        sleep(2000);
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