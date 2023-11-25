package org.firstinspires.ftc.teamcode.garbage;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TICKS_PER_REV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.drive.Pooja;

/*
 * This routine is designed to tune the open-loop feedforward coefficients. Although it may seem unnecessary,
 * tuning these coefficients is just as important as the positional parameters. Like the other
 * manual tuning routines, this op mode relies heavily upon the dashboard. To access the dashboard,
 * connect your computer to the RC's WiFi network. In your browser, navigate to
 * https://192.168.49.1:8080/dash if you're using the RC phone or https://192.168.43.1:8080/dash if
 * you are using the Control Hub. Once you've successfully connected, start the program, and your
 * robot will begin moving forward and backward according to a motion profile. Your job is to graph
 * the velocity errors over time and adjust the feedforward coefficients. Once you've found a
 * satisfactory set of gains, add them to the appropriate fields in the DriveConstants.java file.
 *
 * Pressing Y/Δ (Xbox/PS4) will pause the tuning process and enter driver override, allowing the
 * user to reset the position of the bot in the event that it drifts off the path.
 * Pressing B/O (Xbox/PS4) will cede control back to the tuning process.
 */

@Disabled
@Config
@Autonomous(group = "drive")
public class LinearSlidePIDFeedforwardTuner extends LinearOpMode {
    public static double COUNTS = 400; // counts

    public static double motor_kV = 0.00275;
    public static double motor_kA = 0.0;
    public static double motor_kStatic = 0.0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    Pooja pooja;

    private DcMotorEx collectorTop, collectorBottom;
    private PIDController controller;

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private Mode mode;

    private static MotionProfile generateProfile(boolean extending) {
        MotionState start = new MotionState(extending ? 0 : ((COUNTS / TICKS_PER_REV) * (30 / 25.4)), 0, 0, 0);
        MotionState goal = new MotionState(extending ? ((COUNTS / TICKS_PER_REV) * (30 / 25.4)) : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 20, 20);
    }

    @Override
    public void runOpMode() {
        if (RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }

        pooja = new Pooja(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        controller = new PIDController();

        collectorTop = hardwareMap.get(DcMotorEx.class, "collectorTop");
        collectorBottom = hardwareMap.get(DcMotorEx.class, "collectorBottom");

        mode = Mode.TUNING_MODE;

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        pooja.FBToAfterInitPos();

        waitForStart();

        if (isStopRequested()) return;

        boolean extending = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();


        while (!isStopRequested()) {
            telemetry.addData("mode", mode);

            if (gamepad1.y) {
                mode = Mode.DRIVER_MODE;
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
            double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), motor_kV, motor_kA, motor_kStatic); // controller.calculate(collectorTop.getCurrentPosition(), COUNTS, motor_kP, motor_kI, motor_kD);

            collectorTop.setPower(targetPower);
            collectorBottom.setPower(targetPower);

            double currentVelo = collectorTop.getVelocity() / TICKS_PER_REV;
            double currentPos = collectorTop.getCurrentPosition();

            // update telemetry
            telemetry.addData("targetVelocity", motionState.getV());
            telemetry.addData("measuredVelocity", currentVelo);
            telemetry.addData("targetPosition", COUNTS);
            telemetry.addData("measuredPosition", collectorTop.getCurrentPosition());
            telemetry.addData("error", motionState.getV() - currentVelo);

            telemetry.update();
        }
    }
}