package org.firstinspires.ftc.teamcode.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Pooja;

public class PIDController {
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kF = 0.0;
    public double maxPower = 1.0;

    ElapsedTime clock = new ElapsedTime();

    double currentError = 0.0;
    double previousError = 0.0;

    private double p = 0.0;
    private double i = 0.0;
    private double d = 0.0;
    private double integralSum = 0.0;
    private double maxIntegralSum = integralSum;

    private double ticksPerMM = 0.82;

    public PIDController(double kP, double kI, double kD, double maxPower) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxPower = maxPower;
    }

    public PIDController(double kP, double kI, double kD, double kF, double maxPower) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.maxPower = maxPower;
    }

    public double calculateUsingEncoder(double target, double current) {
        currentError = target - current;

        p = kP * currentError;

        integralSum = integralSum + (currentError * clock.seconds());
        if (Math.abs(integralSum) < Math.abs(maxIntegralSum)) {
            integralSum *= -(maxIntegralSum - integralSum);
        }
        i = integralSum * kI;

        d = kD * (currentError - previousError) / clock.seconds();
        previousError = currentError;

        maxIntegralSum = integralSum;
        clock.reset();

        double targetPower = p + i + d;
        return (Math.abs(targetPower) > maxPower ? (targetPower >= 0 ? maxPower : -maxPower) : targetPower);
    }

    public double calculateUsingDistanceSensor(double reading) {
        // calculate the error
        currentError = reading * 0.82 - Pooja.collectableConeDistance;

        p = kP * currentError;

        // rate of change of error, d value
        d = kD * (currentError - previousError) / clock.seconds();
        previousError = currentError;

        // accumulated error
        integralSum = integralSum + (currentError * clock.seconds());
        if (Math.abs(integralSum) < Math.abs(maxIntegralSum)) {
            integralSum *= -(maxIntegralSum - integralSum);
        }
        i = integralSum * kI;

        maxIntegralSum = integralSum;
        clock.reset();

        double targetPower = p + i + d;
        return (Math.abs(targetPower) > maxPower ? (targetPower >= 0 ? maxPower : -maxPower) : targetPower);
    }


    public double getIntegralSum() {
        return integralSum;
    }

    public double getCurrentError() {
        return currentError;
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }
}
