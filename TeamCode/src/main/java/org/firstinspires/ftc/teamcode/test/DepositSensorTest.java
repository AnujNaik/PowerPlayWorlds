//package org.firstinspires.ftc.teamcode.test;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//
//import org.firstinspires.ftc.teamcode.drive.Pooja;
//
//@TeleOp(group = "Test")
//public class DepositSensorTest extends LinearOpMode {
////    private DcMotorEx collectorTop, collectorBottom;
//    private DistanceSensor depositSensorDistance;
//    private ColorSensor depositSensorColor;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Pooja pooja = new Pooja(hardwareMap);
//
//        depositSensorDistance = hardwareMap.get(DistanceSensor.class, "depositSensor");
//        depositSensorColor = hardwareMap.get(ColorSensor.class, "depositSensor");
//
//        waitForStart();
//
//        while (!isStopRequested()) {
////            telemetry.addData("Deposit sensor color reading", pooja.getDepositSensorColorReading());
////            telemetry.addData("Deposit sensor distance reading", pooja.getDepositSensorDistanceReading());
//            telemetry.update();
//        }
//    }
//}
