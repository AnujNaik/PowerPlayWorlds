package org.firstinspires.ftc.teamcode.PID;


import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Pooja;


@Disabled
@Config
@Autonomous(group = "drive")
public class PIDTuner extends LinearOpMode {
    public static double COUNTS = 400; // counts

    PIDController controller;
    public static double deposit_kP = 0.02;
    public static double deposit_kI = 0.0;
    public static double deposit_kD = 0.0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    Pooja pooja;

    private DcMotorEx collectorTop, collectorBottom;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {
        pooja = new Pooja(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        collectorTop = hardwareMap.get(DcMotorEx.class, "collectorTop");
        collectorBottom = hardwareMap.get(DcMotorEx.class, "collectorBottom");

        collectorTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collectorBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(deposit_kP, deposit_kI, deposit_kD, 1.0);

        waitForStart();

        while (opModeIsActive() && COUNTS - Math.abs(collectorTop.getCurrentPosition()) > 10) {
            double targetPower = controller.calculateUsingEncoder(COUNTS, collectorTop.getCurrentPosition());
            collectorTop.setPower(targetPower);
            collectorBottom.setPower(targetPower);
            telemetry.addData("Pos", collectorTop.getCurrentPosition());
            telemetry.update();
        }

        collectorTop.setPower(0.1);
        collectorBottom.setPower(0.1);
    }
}
