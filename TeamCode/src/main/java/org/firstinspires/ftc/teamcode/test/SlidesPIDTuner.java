package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Pooja;

@Disabled
@Config
@TeleOp
public class SlidesPIDTuner extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private DcMotorEx dropperFront, dropperBack;
    Pooja pooja;

    public static double kP = 0.097, kI = 0.015, kD = 0.0001;
    public static double reference = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        pooja = new Pooja(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        dropperFront = hardwareMap.get(DcMotorEx.class, "collectorBottom");
        dropperBack = hardwareMap.get(DcMotorEx.class, "collectorTop");

        dropperFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropperFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dropperBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropperBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Target Position", reference);
        telemetry.addData("Position", dropperBack.getCurrentPosition());
        telemetry.addData("Output", 0.0);
        telemetry.update();

        PIDController controller = new PIDController(kP, kI, kD);
        controller.setSetPoint(reference);

        waitForStart();

        while (!isStopRequested()) {
            double output = controller.calculate(
                    dropperBack.getCurrentPosition(), reference
            );
            dropperBack.setPower(output);
            dropperFront.setPower(output);

            telemetry.addData("Target Position", reference);
            telemetry.addData("Output", output);
            telemetry.addData("Position", dropperBack.getCurrentPosition());
            telemetry.update();

//            if (dropperBack.getCurrentPosition() == reference) {
//                dropperBack.setPower(0.05);
//                dropperFront.setPower(0.05);
//                break;
//            }
        }

//        sleep(5000);

        dropperBack.setPower(0);
        dropperFront.setPower(0);
    }
}
