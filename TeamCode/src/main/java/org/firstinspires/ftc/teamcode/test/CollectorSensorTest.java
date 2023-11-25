package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.drive.Pooja;

@Disabled
@TeleOp(group = "Test")
public class CollectorSensorTest extends LinearOpMode {
    private DcMotorEx collectorTop, collectorBottom;
    private DistanceSensor collectorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pooja pooja = new Pooja(hardwareMap);
        pooja.FBToIntake(1);

        collectorTop = hardwareMap.get(DcMotorEx.class, "collectorTop");
        collectorBottom = hardwareMap.get(DcMotorEx.class, "collectorBottom");
        collectorSensor = hardwareMap.get(DistanceSensor.class, "collectorSensor");

//        collectorTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        collectorTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        collectorTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        collectorBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        collectorBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        collectorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            double pow = gamepad1.left_stick_y;

            collectorTop.setPower(pow);
            collectorBottom.setPower(pow);

            telemetry.addData("Position", collectorTop.getCurrentPosition());
            telemetry.addData("Power", pow);
            telemetry.addData("Collector sensor reading", pooja.getCollectorSensorReading());
            telemetry.update();
        }
    }
}
