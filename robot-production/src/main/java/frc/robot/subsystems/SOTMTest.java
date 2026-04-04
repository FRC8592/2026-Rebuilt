package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Pair;
import frc.robot.Constants.SCORING;

public class SOTMTest{
public static void main(String[] args) {
    for (double k = 0; k < 6; k = k + 0.1){
        double xy = Math.sqrt((Math.pow(k, 2) / (2)));
        Pair<Double,Double> temp = SOTM(xy, xy, 0, 0);
        System.out.println(k + "," + temp.getFirst());
    }
}
    
    private static double kFactor = 0.925;
    private static double kAdjustment = 0.73; //0.73
    static double initialAngle = 64; //degrees
    static double hubHeight = 6; //feet
    static double initialBallHeight = 2.18; //feet
    static double g = 32.174; //feet per s^2
    static double flywheelRadius = 2.0; //inches
    static double flywheelGearing = 1.0;
    static double feetPerMeter = 3.28084;

    public static double solveV0y(
            double thetaRad,
            double x,      // radial distance to target
            double vRx,    // robot tangential velocity
            double vRy,    // robot radial velocity toward target
            double hI,
            double hF,
            double g
    ) {
        double dh = hF - hI;
        double tanTheta = Math.tan(thetaRad);
    
        DoubleUnaryOperator f = (v0y) -> {
            double denom = v0y + vRy;
            if (denom <= 0.0) {
                return Double.NaN; // invalid physically
            }
    
            double horizSpeedRelativeRobot = Math.sqrt(v0y * v0y + vRx * vRx);
    
            return tanTheta * horizSpeedRelativeRobot * x / denom
                    - 0.5 * g * x * x / (denom * denom)
                    - dh;
        };
    
        // Start just above the singularity v0y = -vRy
        double lower = -vRy + 1e-9;
        if (lower <= 0.0) {
            lower = 1e-9;
        }
    
        // Search for a sign change by scanning upward.
        double upper = lower + Math.max(10.0, Math.abs(x) * 10.0 + 10.0);
        double left = lower;
        double fLeft = f.applyAsDouble(left);
    
        for (int expand = 0; expand < 20; expand++) {
            double prevU = left;
            double prevF = fLeft;
    
            int samples = 2000;
            for (int i = 1; i <= samples; i++) {
                double u = lower + (upper - lower) * i / samples;
                double fu = f.applyAsDouble(u);
    
                if (Double.isFinite(prevF) && Double.isFinite(fu)) {
                    if (prevF == 0.0) {
                        return prevU;
                    }
                    if (prevF * fu <= 0.0) {
                        return bisect(f, prevU, u, 1e-5, 100);
                    }
                }
    
                prevU = u;
                prevF = fu;
            }
    
            // No root in this interval; expand and try again.
            upper *= 2.0;
        }
    
        throw new IllegalArgumentException("No physical root found for v0y.");
    }
    
    private static double bisect(DoubleUnaryOperator f, double a, double b, double tol, int maxIter) {
        double fa = f.applyAsDouble(a);
        double fb = f.applyAsDouble(b);

        if (!Double.isFinite(fa) || !Double.isFinite(fb)) {
            throw new IllegalArgumentException("Invalid bracket endpoints.");
        }
        if (fa * fb > 0.0) {
            throw new IllegalArgumentException("Bisection requires a sign change.");
        }

        for (int i = 0; i < maxIter; i++) {
            double m = 0.5 * (a + b);
            double fm = f.applyAsDouble(m);

            if (!Double.isFinite(fm)) {
                // Nudge away from singularities if needed.
                m = Math.nextUp(m);
                fm = f.applyAsDouble(m);
            }

            if (Math.abs(fm) < tol || 0.5 * (b - a) < tol) {
                return m;
            }

            if (fa * fm <= 0.0) {
                b = m;
                fb = fm;
            } else {
                a = m;
                fa = fm;
            }
        }

        return 0.5 * (a + b);
    }

    /**
     * Shoot on the Move
     * Returns a pair of flywheel motor RPM and turret angle
     * Turret angle (degrees) is relative to the field, counterclockwise from x-axis
     * If no solutions exist, outputs (0, 0)
     */

    public static Pair<Double, Double> SOTM(double targetX, double targetY, double robotVelX, double robotVelY) {
        double thetaDeg = initialAngle;
        double thetaRad = Math.toRadians(thetaDeg);
        double targetXFeet = targetX * feetPerMeter;
        double targetYFeet = targetY * feetPerMeter;


        double x = Math.sqrt(targetXFeet*targetXFeet + targetYFeet*targetYFeet);

        double angleToHub = Math.atan2(targetYFeet, targetXFeet);
        double vRx = robotVelX * Math.sin(angleToHub) + robotVelY * Math.cos(angleToHub);
        double vRy = robotVelX * Math.cos(angleToHub) + robotVelY * Math.sin(angleToHub);
        double hI = initialBallHeight;
        double hF = hubHeight;
        double g = 32.174;
        try {
            double v0y = solveV0y(thetaRad, x, vRx, vRy, hI, hF, g);
            double v0x = -vRx; // if tangential motion is being canceled
            double totalSpeedRelativeRobot = Math.sqrt(v0x * v0x + v0y * v0y)*Math.sqrt(1 + Math.tan(thetaRad)*Math.tan(thetaRad));
            double flyRadiusFeet = flywheelRadius / 12.0;
            double adjustedK = kFactor + kAdjustment*x/feetPerMeter;
            if(x/feetPerMeter < 2){
                adjustedK = 2.3;
            }
            double outputRPM = adjustedK*(totalSpeedRelativeRobot/flyRadiusFeet)*(60.0/(2*Math.PI));

            double turretAngleToHub = Math.atan2(v0y, v0x);
            Logger.recordOutput(SCORING.LOG_PATH +"Turret Angle To Hub Based on Hub Coordinate Systems", turretAngleToHub);

            double turretFieldAngle = (((Math.toDegrees(turretAngleToHub) - (90.0 - Math.toDegrees(angleToHub))))%360+360)%360;
            Logger.recordOutput(SCORING.LOG_PATH + "Field Angle Offset for Turret", 90 - Math.toDegrees(angleToHub));

            return new Pair<>(outputRPM*flywheelGearing, turretFieldAngle);
        } catch (IllegalArgumentException e) {
            return new Pair<>(0.0, 0.0);
        }
    }
}
