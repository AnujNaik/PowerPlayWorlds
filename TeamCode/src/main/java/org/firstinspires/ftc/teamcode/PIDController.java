package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.util.NanoClock;

public class PIDController {
    private static double currentTime;
    private static double previousTime = 0.0;
    private static double currentError;
    private static double previousError = 0.0;
    private static double maxI = 0.0;
    NanoClock clock = NanoClock.system();

    public double[] calculate(double position, double target, double kP, double kD, double time) {
        position = Math.abs(position);
        currentError = target - position;

        double p = kP * currentError;

        double i = (currentError * time);
//
//        if (i > maxI) {
//            maxI = i;
//        }
//        else if (i < -maxI) {
//            maxI = -i;
//        }

        double d = kD * (currentError - previousError) / time;

        previousTime = currentTime;
        previousError = currentError;

        return new double[]{p + d, i};
    }
}
