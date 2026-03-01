package frc.robot.subsystems;

import org.ejml.equation.IntegerSequence.Range;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.*;

public class RangeTable {
    public final static RangeEntry[] RANGE_TO_RPM = {
            new RangeEntry(2000.0),    // 0.0m
            new RangeEntry(2100.0),    // 0.2m
            new RangeEntry(2200.0),    // 0.4m
            new RangeEntry(2300.0),    // 0.6m
            new RangeEntry(2400.0),    // 0.8m
            new RangeEntry(2500.0),    // 1.0m
            new RangeEntry(2600.0),    // 1.2m
            new RangeEntry(2700.0),    // 1.4m
            new RangeEntry(2800.0),    // 1.6m 
            new RangeEntry(2900.0),    // 1.8m 
            new RangeEntry(2000.0),    // 2.0m 
            new RangeEntry(2100.0),    // 2.2m 
            new RangeEntry(2200.0),    // 2.4m 
            new RangeEntry(2300.0),    // 2.6m 
            new RangeEntry(2400.0),    // 2.8m 
            new RangeEntry(2500.0),    // 3.0m 
            new RangeEntry(2600.0),    // 3.2m 
            new RangeEntry(2700.0),    // 3.4m 
            new RangeEntry(2800.0),    // 3.6m 
            new RangeEntry(2900.0),    // 3.8m 
            new RangeEntry(3000.0),    // 4.0m
            new RangeEntry(3100.0),    // 4.2m
            new RangeEntry(3200.0),    // 4.4m
            new RangeEntry(3300.0),    // 4.6m
            new RangeEntry(3400.0),    // 4.8m
            new RangeEntry(3500.0),    // 5.0m
            new RangeEntry(3600.0),    // 5.2m
            new RangeEntry(3700.0),    // 5.4m
            new RangeEntry(3800.0),    // 5.6m
            new RangeEntry(3900.0),    // 5.8m
            new RangeEntry(4000.0),    // 6.0m
            new RangeEntry(4100.0),    // 6.2m
            new RangeEntry(4200.0),    // 6.4m
            new RangeEntry(4300.0),    // 6.6m
            new RangeEntry(4400.0),    // 6.8m
            new RangeEntry(4500.0),    // 7.0m 
            new RangeEntry(4600.0),    // 7.2m 
            new RangeEntry(4700.0),    // 7.4m 
            new RangeEntry(4900.0),    // 7.6m 
            new RangeEntry(5000.0),    // 7.8m 
            new RangeEntry(5100.0),    // 8.0m 
            new RangeEntry(5200.0),    // 8.2m 
            new RangeEntry(5300.0),    // 8.4m 
            new RangeEntry(5400.0),    // 8.6m 
            new RangeEntry(5500.0),    // 8.8m 
            new RangeEntry(5500.0),    // 9.0m 
            new RangeEntry(5500.0),    // 9.2m 
            new RangeEntry(5500.0),    // 9.4m
            new RangeEntry(5500.0),    // 9.6m
            new RangeEntry(5500.0),    // 9.8m
            new RangeEntry(5500.0),    // 10.0m
    };

    public RangeTable() {
    }

    /**
     * @param distance in meters
     * @return flywheel speed in RPM from the RangeTable
     */
    public static double get(double distance) {
        Logger.recordOutput("CustomLogs/RangeTable/InputDistance", distance);
        if(distance <= 0){
            return 0.0;
        }
        if (distance < RANGE_TABLE.MAX_TABLE_DISTANCE) {
            return interpolate(distance);
        }
        else{
            return (RANGE_TO_RPM[RANGE_TO_RPM.length - 1]).getFlywheelSpeed();
        }
    }

    /**
     * @param distance in meters
     * @return flywheel speed in RPM
     */
    public static double interpolate(double distance){
        int lowEndEntry = (int)(distance / RANGE_TABLE.RANGE_TABLE_STEP);
        int highEndEntry = lowEndEntry + 1;

        double x1 = lowEndEntry * RANGE_TABLE.RANGE_TABLE_STEP;
        double x2 = highEndEntry * RANGE_TABLE.RANGE_TABLE_STEP;

        double y1 = RANGE_TO_RPM[lowEndEntry].getFlywheelSpeed();
        double y2 = RANGE_TO_RPM[highEndEntry].getFlywheelSpeed();

        double interpolationValue = y1 + (distance - x1) * ((y2 - y1) / (x2 - x1));

        return interpolationValue;

    }

    public static class RangeEntry {
        public double flywheelSpeed;

        public RangeEntry(double RPM){
            flywheelSpeed = RPM;
        }

        public double getFlywheelSpeed(){
            return flywheelSpeed;
        }
    }
}