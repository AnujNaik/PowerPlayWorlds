package org.firstinspires.ftc.teamcode.garbage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@Config
@TeleOp
public class FeedforwardSlidesTuner extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
//
    public static double targetPos = 500;

    private DcMotorEx dropperBack;
    private DcMotorEx dropperFront;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        dropperBack = hardwareMap.get(DcMotorEx.class, "dropperBack");
        dropperFront = hardwareMap.get(DcMotorEx.class, "dropperFront");

        dropperBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropperBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dropperFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropperFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFController collectorSlidesPID = new PIDFController(kP, kI, kD, kF);
        collectorSlidesPID.setSetPoint(targetPos);
        collectorSlidesPID.setTolerance(5, 10);

        waitForStart();

        while (!isStopRequested()) {
            double output = collectorSlidesPID.calculate(
                    dropperBack.getCurrentPosition(), targetPos
            );
            dropperBack.setVelocity(output);
            dropperFront.setVelocity(output);
            telemetry.addData("Position", dropperBack.getCurrentPosition());

            if (collectorSlidesPID.atSetPoint()) {
                telemetry.addData("Reached Position", dropperBack.getCurrentPosition());
            }

            telemetry.update();
        }

//        ElapsedTime timer = new ElapsedTime();
//        while (!isStopRequested()) {
//            slidesPos = collectorTop.getCurrentPosition();
//            error = targetPos - slidesPos;
//            derivative = (error - lastError) / timer.seconds();
//            integralSum = integralSum + (error * timer.seconds());
//
//            output = (Kp * error) + (Ki * integralSum) + (Kd * derivative) Kv * (referenceSpeed) + Ka * referenceAccel;
//        }

        /**
         *
         * // obtain the encoder position
         *     encoderPosition = armMotor.getPosition();
         *     // calculate the error
         *     error = reference - encoderPosition;
         *
         *     // rate of change of the error
         *     derivative = (error - lastError) / timer.seconds();
         *
         *     // sum of all error over time
         *     integralSum = integralSum + (error * timer.seconds());
         *
         *     out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
         *     double output = PID(referenceSpeed, currentSpeed) + Kv * (referenceSpeed) + Ka * referenceAccel;
         *
         *     armMotor.setPower(out);
         *
         *     lastError = error;
         *
         *     // reset the timer for next time
         *     timer.reset();
         */
    }
}
