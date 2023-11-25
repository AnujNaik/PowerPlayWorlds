package org.firstinspires.ftc.teamcode.garbage;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TICKS_PER_REV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.Pooja;

@Disabled
@TeleOp
@Config
public class UpdatedDropperSlidesPIDTest extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    Pooja pooja = new Pooja(hardwareMap);

    public static double COUNTS = 910; // counts
//    public static double kD = 0;
    public static double kV = 0.0;
    public static double kP = 0.0;

    private double lastError = 0;
    private double integralSum = 0;

    private DcMotorEx collectorTop, collectorBottom;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        collectorTop = hardwareMap.get(DcMotorEx.class, "collectorTop");
        collectorBottom = hardwareMap.get(DcMotorEx.class, "collectorBottom");

        collectorTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collectorBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("measuredVelocity", collectorBottom.getVelocity());
        telemetry.addData("current position", collectorBottom.getCurrentPosition());
        telemetry.addData("target position", COUNTS);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

            MotorPIDTuner.Mode mode = MotorPIDTuner.Mode.TUNING_MODE;

            NanoClock clock = NanoClock.system();

            telemetry.addLine("Ready!");
            telemetry.update();
            telemetry.clearAll();

            waitForStart();

            if (isStopRequested()) return;

            boolean extending = true;
            MotionProfile activeProfile = generateProfile(true);
            double profileStart = clock.seconds();


            while (!isStopRequested()) {
                telemetry.addData("mode", mode);

                if (gamepad1.y) {
                    mode = MotorPIDTuner.Mode.DRIVER_MODE;
                }

                // calculate and set the motor power
                double profileTime = clock.seconds() - profileStart;

                if (profileTime > activeProfile.duration()) {
                    // generate a new profile
                    extending = !extending;
                    activeProfile = generateProfile(extending);
                    profileStart = clock.seconds();
                }

                MotionState motionState = activeProfile.get(profileTime);
                double targetPower = PIDControl(COUNTS, motionState.getV(), collectorBottom);
                //            double targetPower = controller.update(motor2.getCurrentPosition(), motor2.getVelocity(AngleUnit.RADIANS));

                collectorBottom.setPower(targetPower);
                collectorTop.setPower(targetPower);

                double currentVelo = collectorTop.getVelocity();

                // update telemetry
                telemetry.addData("targetVelocity", motionState.getV());
                telemetry.addData("measuredVelocity", currentVelo);
                telemetry.addData("error", motionState.getV() - currentVelo);

                telemetry.update();

                telemetry.addData("measuredVelocity", collectorTop.getVelocity());
                telemetry.addData("current position", collectorTop.getCurrentPosition());
                telemetry.addData("target position", COUNTS);
                telemetry.update();
            }
        }
    }

    private static MotionProfile generateProfile(boolean extending) {
        MotionState start = new MotionState(extending ? 0 : ((COUNTS / TICKS_PER_REV) * (30 / 25.4)), 0, 0, 0);
        MotionState goal = new MotionState(extending ? ((COUNTS / TICKS_PER_REV) * (30 / 25.4)) : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 30, 30);
    }

    public double PIDControl(double referencePosition, double referenceVelocity, DcMotorEx motor) {
//        double error = reference - state;
//        integralSum += error * timer.seconds();
//
//        double derivative = (error - lastError) / timer.seconds();
//        lastError = error;
//
//        timer.reset();
//
//        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
//        return output;

        double positionError = referencePosition - motor.getCurrentPosition();
        double velocityError = referenceVelocity - motor.getVelocity();
        return (positionError * kP) + (velocityError * kV);
    }
}
