package org.firstinspires.ftc.teamcode.garbage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
//import static org.firstinspires.ftc.teamcode.drive.Pooja.motor_kV;
//import static org.firstinspires.ftc.teamcode.drive.Pooja.motor_kStatic;
//import static org.firstinspires.ftc.teamcode.drive.Pooja.motor_kA;

@Disabled
@Config
@Autonomous(group = "drive")
public class MotorPIDTunerV2 extends LinearOpMode {
    DcMotorEx dropperFront;
    DcMotorEx dropperBack;

    public static double motor_kV = 0.0004;
    public static double motor_kA = 0.0;
    public static double motor_kStatic = 0.0;

    double integral = 0;
    double repetitions = 0;

    public static PIDCoefficients testPID = new PIDCoefficients(motor_kV, motor_kStatic, motor_kA);

    FtcDashboard dashboard;

    public static double TARGET_POS = 300; // 100 is default value

    ElapsedTime PIDTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        dropperFront = hardwareMap.get(DcMotorEx.class, "dropperFront");
        dropperBack = hardwareMap.get(DcMotorEx.class, "dropperBack");

        dropperFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropperBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dropperFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dropperBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        moveTestMotor(TARGET_POS);
        sleep(4000);
        telemetry.addData("Current position", dropperBack.getCurrentPosition());
        telemetry.update();


//
//        while (!isStopRequested()) {
//
//        }
    }

    public void moveTestMotor(double targetPosition) {
        double error = dropperBack.getCurrentPosition();
        double lastError = 0;
        /*
         * Comparison value dependent on motor tick count
         * Higher end motor tick count: higher value
         * Lower end motor tick count: lower value
         */
        while (error >= 9 /*Modify with above comments*/ && repetitions < 100 /*Modify*/) {
            error = targetPosition - dropperBack.getCurrentPosition();
            double changeInError = lastError - error;
            integral += changeInError * PIDTimer.time();
            double derivative = changeInError / PIDTimer.time();
            double P = motor_kV * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            dropperFront.setPower((P + I + D));
            dropperBack.setPower((P + I + D));
            error = lastError;
            PIDTimer.reset();
            repetitions++;
        }
    }
}