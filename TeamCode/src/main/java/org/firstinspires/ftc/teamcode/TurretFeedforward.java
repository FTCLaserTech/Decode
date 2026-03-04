package org.firstinspires.ftc.teamcode;

/**
 * Feedforward calculator for a turret with a spring pulling toward zero position.
 *
 * Model:
 *   torque = kSpring * positionRad
 *          + kV * velocityRadPerSec
 *          + kA * accelRadPerSec2
 *          + kS * sign(velocity)
 */
public class TurretFeedforward
{
    private final double motorTorquePerVolt = 0.308;
    private final double kSpring; // Nm/rad
    private final double kS;      // Nm (static friction)
    private final double kV;      // Nm per rad/s
    private final double kA;      // Nm per rad/s^2

    /**
     * @param kSpring Spring constant (Nm/rad)
     * @param kS Static friction torque (Nm)
     * @param kV Velocity gain (Nm per rad/s)
     * @param kA Acceleration gain (Nm per rad/s^2)
     */
    public TurretFeedforward(double kSpring, double kS, double kV, double kA) {
        this.kSpring = kSpring;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Calculates the feedforward torque.
     *
     * @param positionRad Turret angle from zero (radians)
     * @param velocityRadPerSec Angular velocity (rad/s)
     * @param accelRadPerSec2 Angular acceleration (rad/s^2)
     * @return Feedforward torque (Nm)
     */
    public double calculate(double positionRad, double velocityRadPerSec, double accelRadPerSec2) {
        double springTorque = kSpring * positionRad; // maybe this is sin(positionRad)
        double velocityTorque = kV * velocityRadPerSec;
        double accelTorque = kA * accelRadPerSec2;

        // Static friction only applied if moving
        double staticTorque = (Math.abs(velocityRadPerSec) > 1e-4)
                ? Math.copySign(kS, velocityRadPerSec)
                : 0.0;

        return springTorque + velocityTorque + accelTorque + staticTorque;
    }

    /**
     * Overload for when acceleration is not provided (assumed zero).
     */
    public double calculate(double positionRad, double velocityRadPerSec) {
        return calculate(positionRad, velocityRadPerSec, 0.0);
    }
}

