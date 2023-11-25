package org.firstinspires.ftc.teamcode.PID;


import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Pooja;

@Disabled
@Autonomous(group = "drive")
public class PIDTunerRaghav extends LinearOpMode {
    public static double COUNTS; // counts

    PIDController controller;
    public static double collector_kP = 0.0;
    public static double collector_kI = 0.0;
    public static double collector_kD = 0.0;
    public static double collector_kF = 0.0;

    private double p = 0.0;
    private double i = 0.0;
    private double d = 0.0;
    private double integralSum = 0.0;
    private double maxIntegralSum = integralSum;
    private double targetPower = 0.0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    Pooja pooja;

    private DcMotorEx rightRear, rightFront, leftRear, leftFront;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {
        pooja = new Pooja(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");

        ElapsedTime clock = new ElapsedTime();
        controller = new PIDController(collector_kP, collector_kI, collector_kD, collector_kF);

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

//        pooja.FBToAfterInitPos();
        p = 0.0;
        i = 0.0;
        d = 0.0;
        integralSum = 0;
        maxIntegralSum = integralSum;

        telemetry.addData("targetPosition", COUNTS);
        telemetry.addData("measuredPosition", rightRear.getCurrentPosition());
        telemetry.addData("error", COUNTS - rightRear.getCurrentPosition());
        telemetry.addData("Integral sum", integralSum);
        telemetry.update();

//        pooja.FBToIntake(5);

        sleep(1000);

        waitForStart();

        double currentError;
        double previousError = 0.0;


        while (!isStopRequested()) {
            // calculate and set the motor power
//            double position = Math.abs(collectorTop.getCurrentPosition());
            double position = rightRear.getCurrentPosition();

            // calculate the error
            currentError = position;

            p = collector_kP * currentError;

            // rate of change of error, d value
            d = collector_kD * (currentError - previousError) / clock.seconds();
            previousError = currentError;

            // accumulated error
            integralSum = integralSum + (currentError * clock.seconds());
            if (Math.abs(integralSum) < Math.abs(maxIntegralSum)) {
                integralSum *= -(maxIntegralSum - integralSum);
            }
            i = integralSum * collector_kI;


            // sum up all PID values to find desired power
            targetPower = p + i + d + collector_kF;
            rightRear.setPower(targetPower);
            rightFront.setPower(targetPower);
            leftRear.setPower(targetPower);
            leftFront.setPower(targetPower);


            // update telemetry
            telemetry.addData("targetPosition", COUNTS);
            telemetry.addData("measuredPosition", position);
            telemetry.addData("error", currentError);
            telemetry.addData("Integral sum", integralSum);
            telemetry.update();

            maxIntegralSum = integralSum;
            clock.reset();
        }
    }
}
