package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Pooja;

@Config
@TeleOp(name = "InkOp")
public class

InkOp extends LinearOpMode {

    private String level;
    private int stackLevelIntake;
    private Pooja pooja;
    private Thread collectCone, collectConeLow, collectConeGround, depositConeHigh, depositConeMid, depositConeLow, depositConeGround, depositConeCycles, collectConeCycles, transferConeCycles;
    private boolean isFirstCycle = true;

    @Override
    public void runOpMode() throws InterruptedException {
        pooja = new Pooja(hardwareMap);
        collectCone = new CollectCone("high");
        collectConeLow = new CollectCone("low");
        collectConeGround = new CollectCone("ground");
        depositConeHigh = new DepositCone("high");
        depositConeMid = new DepositCone("mid");
        depositConeLow = new DepositCone("low");
        depositConeGround = new DepositCone("ground");
        depositConeCycles = new DepositConeCycles();
        collectConeCycles = new CollectConeCycles();
        transferConeCycles = new TransferConeCycles();
//        autoCollectCone = new AutoCollectCone();
//        collectConeCyclesFirst = new CycleConeCollectionFirst(3);
//        collectConeCyclesSecond = new CycleConeCollectionSecond(1);

        level = "high";
        stackLevelIntake = 1;
        boolean firstDone = false;

        double turnConstant = pooja.coarseTurnConstant;

        waitForStart();

        while (!isStopRequested()) {
            pooja.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.7,
                            -gamepad1.left_stick_x * 0.7,
                            -gamepad1.right_stick_x * turnConstant
                    )
            );

            pooja.update();

            if (gamepad1.a) {
                if (turnConstant == pooja.coarseTurnConstant) {
                    turnConstant = pooja.fineTurnConstant;
                }
                else {
                    turnConstant = pooja.coarseTurnConstant;
                }
            }
            if (gamepad2.dpad_up) {
                level = "high";
            }
            if (gamepad2.dpad_left) {
                level = "mid";
            }
            if (gamepad2.dpad_down) {
                level = "low";
            }
            if (gamepad2.dpad_right) {
                // also for terminal
                level = "ground";
            }

            if (gamepad2.right_bumper) {
                stackLevelIntake = 1;
            }
            if (gamepad2.y) {
                stackLevelIntake = 2;
            }
            if (gamepad2.x) {
                stackLevelIntake = 3;
            }
            if (gamepad2.a) {
                stackLevelIntake = 4;
            }
            if (gamepad2.b) {
                stackLevelIntake = 5;
            }
            if (gamepad2.left_bumper) {
                pooja.engageConeStabilizer();
            }
            if (gamepad2.left_trigger > 0.2) {
                pooja.disengageConeStabilizer();
            }

            /**
             * Collection
             */
            if (gamepad1.dpad_up) {
                pooja.FBToRest();
                pooja.openClaw();
                pooja.rotateClawToIntakeTeleOp();
            }
            if (gamepad1.left_bumper && !collectCone.isAlive()) {
                pooja.FBToIntakeTele();
                pooja.openClaw();
                pooja.rotateClawToIntakeTeleOp();
            }
            if (gamepad1.y) {
                pooja.closeClawTele();
                sleep(150);
                pooja.FBToTransfer();
                pooja.rotateClawToTransfer();

                pooja.retractCollectorSlides("TELE");

                sleep(500);
                pooja.openClawAuto();
                sleep(500);
                pooja.FBToRest();
                sleep(300);
                pooja.openClaw();
            }

            if (gamepad1.left_trigger > 0.2 && !collectCone.isAlive() && !collectConeLow.isAlive() && !collectConeGround.isAlive()) {
                if (level.equals("high") || level.equals("mid")) {
                    collectCone.start();
//                    pooja.collectCone();
                } else if (level.equals("low")) {
                    collectConeLow.start();
                } else if (level.equals("ground")) {
                    collectConeGround.start();
                }
            }

            if (!collectCone.isAlive() && !collectConeCycles.isAlive() && !transferConeCycles.isAlive()) {
                float leftStickY = gamepad2.left_stick_y;
                leftStickY = (float) scaleInput(leftStickY);
                pooja.setCollectorSlidesPower(leftStickY);
            }

            /**
             * Deposit
             */
            if (gamepad1.right_trigger > 0.2 && !collectCone.isAlive()) {
                if (level.equals("high") && !depositConeHigh.isAlive()) {
                    depositConeHigh.start();
                }
                if (level.equals("mid") && !depositConeMid.isAlive()) {
                    depositConeMid.start();
                }
                if (level.equals("low") && !depositConeLow.isAlive()) {
                    depositConeLow.start();
                }
                if (level.equals("ground") && !depositConeGround.isAlive()) {
                    depositConeGround.start();
                }
            }

            /**
             * Cycles
             */

//            if (opModeIsActive() && gamepad1.x && !collectConeCyclesFirst.isAlive()) {
//                collectConeCyclesFirst.start();
//                firstDone = true;
//            }
//
//            if (opModeIsActive() && !collectConeCyclesFirst.isAlive() && firstDone == true) {
//                collectConeCyclesSecond.start();
//                firstDone = false;
//            }

            if (gamepad1.x) {
                pooja.transferConeCycles();
                depositConeCycles.start();
                pooja.collectConeCycles();
                sleep(300);
                // slides extended, arm down, claw open
                // close claw, transfer cone

                // repeat
                // deposit raise + go to intake
                // deposit drop and go to transfer

                // go to intake, raise, start transfer thread, drop
                // while going to intake, raise the dropper
                // once dropper is finished raising, and the intake is in position, transfer
                // once dropper is finished raising, lower the dropper
                // once transfer is finished, go to intake and raise the dropper

            }

            if (pooja.isFBArmPosToIntake() && pooja.retrieveCollectorEncoderCount() < 300) {
                pooja.FBToIntakeTele(stackLevelIntake, 1);
            }
            if (pooja.isFBArmPosToIntake() && pooja.retrieveCollectorEncoderCount() >= 300) {
                pooja.FBToIntakeTele(stackLevelIntake, 2);
            }
            if (pooja.isFBArmPosToIntake() && pooja.retrieveCollectorEncoderCount() >= 600) {
                pooja.FBToIntakeTele(stackLevelIntake, 3);
            }


            telemetry.addData("Level", level);
            telemetry.addData("Stack Level Intake", stackLevelIntake);
//            telemetry.addData("Dropper Count", pooja.retrieveDropperEncoderCount());
//            telemetry.addData("Collector Count", pooja.retrieveCollectorEncoderCount());
            telemetry.addData("Turn mode", turnConstant);
            telemetry.update();

        }
    }

    private class CollectCone extends Thread {
        String level;

        public CollectCone(String level) {
            this.level = level;
        }

        @Override
        public void run() {
            try {
                if (level.equals("high") || level.equals("mid")) {
                    pooja.collectCone();
                } else if (level.equals("low")) {
                    pooja.closeClawTele();
                    sleep(200);
                    pooja.FBToLowJunction();
                    sleep(1000);
                } else if (level.equals("ground")) {
                    pooja.closeClawTele();
                    sleep(200);
                    pooja.FBToGroundRaised();
                    sleep(200);
                }
            } catch (InterruptedException e) {
                return;
            }
            this.interrupt();
        }
    }

    private class DepositCone extends Thread {
        String level;

        public DepositCone(String level) {
            this.level = level;
        }

        @Override
        public void run() {
            try {
                if (level.equals("mid") || level.equals("high")) {
                    pooja.engageConeStabilizer();
                    pooja.raiseDepositSlides(telemetry, level);
                    while (!gamepad1.right_bumper) {
                        float rightStickY = (float) (gamepad2.right_stick_y * 0.3);
                        pooja.setDepositSlidesPower(rightStickY + 0.1);
                    }
                    pooja.lowerDepositSlides(telemetry);
                }
                if (level.equals("low")) {
                    pooja.openClaw();
                }
                if (level.equals("ground")) {
                    pooja.FBToGround();
                    pooja.openClaw();
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            this.interrupt();
        }
    }

    private class DepositConeCycles extends Thread {

        @Override
        public void run() {
            try {
                sleep(150);
                pooja.raiseDepositSlides(telemetry, level);
                pooja.lowerDepositSlides(telemetry);
            } catch (Exception e) {
                e.printStackTrace();
            }
            this.interrupt();
        }
    }

    private class CollectConeCycles extends Thread {

        @Override
        public void run() {
            try {
                pooja.collectConeCycles();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            this.interrupt();
        }
    }

    private class TransferConeCycles extends Thread {

        @Override
        public void run() {
            try {
                pooja.transferConeCycles();
                depositConeCycles.start();
                pooja.collectConeCycles();
                sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            this.interrupt();
        }
    }

//    private class CycleConeCollectionFirst extends Thread {
//        int level;
//
//        public CycleConeCollectionFirst(int level) {
//            this.level = level;
//        }
//
//        @Override
//        public void run() {
//            try {
//                pooja.extendDropperAutoHighTele(3, telemetry, autoCollectCone);
//
//            } catch (Exception e) {
//                // pooja.endExtendCollectorSlides();
//                return;
//            }
//            this.interrupt();
//        }
//    }
//
//    private class CycleConeCollectionSecond extends Thread {
//        int level;
//
//        public CycleConeCollectionSecond(int level) {
//            this.level = level;
//        }
//
//        @Override
//        public void run() {
//            try {
//                sleep(500);
//                pooja.extendDropperAutoHighTele(1, telemetry, autoCollectCone);
//            } catch (Exception e) {
//                return;
//            }
//            this.interrupt();
//        }
//    }
//
//    private class AutoCollectCone extends Thread {
//        public AutoCollectCone() {
//        }
//
//        @Override
//        public void run() {
//            try {
//                pooja.collectCone();
//            } catch (InterruptedException e) {
//            }
//            this.interrupt();
//        }
//    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}


/**
 * collectCone
 * entire collect --> transfer --> arm rest process
 * depositCone (only if cone is collected)
 * collectorSlidesControl
 * thread runs continuously, listens for 2 triggers, if not then keeps slides in with brake
 */