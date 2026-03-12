package frc.robot.subsystems;

import org.ejml.equation.IntegerSequence.Range;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.*;

public class RangeTable {
    public final static RangeEntry[] RANGE_TO_RPM_HUB = {
            new RangeEntry(4650),    // 0.0m
            new RangeEntry(4650),    // 0.2m
            new RangeEntry(4650),    // 0.4m
            new RangeEntry(4650),    // 0.6m
            new RangeEntry(4650),    // 0.8m
            new RangeEntry(4650),    // 1.0m
            new RangeEntry(4650),    // 1.2m
            new RangeEntry(4650),    // 1.4m
            new RangeEntry(4650),    // 1.6m 
            new RangeEntry(4650),    // 1.8m 
            new RangeEntry(4650),    // 2.0m *
            new RangeEntry(4950),    // 2.2m *
            new RangeEntry(5450),    // 2.4m *
            new RangeEntry(5450),    // 2.6m *
            new RangeEntry(5850),    // 2.8m *
            new RangeEntry(6050),    // 3.0m *
            new RangeEntry(6150),    // 3.2m 
            new RangeEntry(6150),    // 3.4m 
            new RangeEntry(6150),    // 3.6m 
            new RangeEntry(6150),    // 3.8m 
            new RangeEntry(6150),    // 4.0m
            new RangeEntry(6150),    // 4.2m
            new RangeEntry(6150),    // 4.4m
            new RangeEntry(6150),    // 4.6m
            new RangeEntry(6150),    // 4.8m
            new RangeEntry(6150),    // 5.0m
            new RangeEntry(6150),    // 5.2m
            new RangeEntry(6150),    // 5.4m
            new RangeEntry(6150),    // 5.6m
            new RangeEntry(6150),    // 5.8m
            new RangeEntry(6150),    // 6.0m
            new RangeEntry(6150),    // 6.2m
            new RangeEntry(6150),    // 6.4m
            new RangeEntry(6150),    // 6.6m
            new RangeEntry(6150),    // 6.8m
            new RangeEntry(6150),    // 7.0m 
            new RangeEntry(6150),    // 7.2m 
            new RangeEntry(6150),    // 7.4m 
            new RangeEntry(6150),    // 7.6m 
            new RangeEntry(6150),    // 7.8m 
            new RangeEntry(6150),    // 8.0m 
            new RangeEntry(6150),    // 8.2m 
            new RangeEntry(6150),    // 8.4m 
            new RangeEntry(6150),    // 8.6m 
            new RangeEntry(6150),    // 8.8m 
            new RangeEntry(6150),    // 9.0m 
            new RangeEntry(6150),    // 9.2m 
            new RangeEntry(6150),    // 9.4m
            new RangeEntry(6150),    // 9.6m
            new RangeEntry(6150),    // 9.8m
            new RangeEntry(6150),    // 10.0m
    };
    //TODO: Change this later for passing and not be a copy of the hub
    public final static RangeEntry[] RANGE_TO_RPM_FLOOR = {
            new RangeEntry(4550),    // 0.0m
            new RangeEntry(4550),    // 0.2m
            new RangeEntry(4550),    // 0.4m
            new RangeEntry(4550),    // 0.6m
            new RangeEntry(4550),    // 0.8m
            new RangeEntry(4550),    // 1.0m
            new RangeEntry(4550),    // 1.2m
            new RangeEntry(4550),    // 1.4m
            new RangeEntry(4550),    // 1.6m 
            new RangeEntry(4550),    // 1.8m 
            new RangeEntry(4550),    // 2.0m
            new RangeEntry(4550),    // 2.2m
            new RangeEntry(4550),    // 2.4m 
            new RangeEntry(4550),    // 2.6m
            new RangeEntry(4550),    // 2.8m 
            new RangeEntry(4550),    // 3.0m 
            new RangeEntry(4550),    // 3.2m
            new RangeEntry(4550),    // 3.4m 
            new RangeEntry(4550),    // 3.6m 
            new RangeEntry(4550),    // 3.8m 
            new RangeEntry(4550),    // 4.0m
            new RangeEntry(4550),    // 4.2m
            new RangeEntry(4550),    // 4.4m
            new RangeEntry(4550),    // 4.6m
            new RangeEntry(4550),    // 4.8m
            new RangeEntry(4550),    // 5.0m
            new RangeEntry(4550),    // 5.2m
            new RangeEntry(4550),    // 5.4m
            new RangeEntry(4550),    // 5.6m
            new RangeEntry(4550),    // 5.8m
            new RangeEntry(4550),    // 6.0m
            new RangeEntry(4550),    // 6.2m
            new RangeEntry(4550),    // 6.4m
            new RangeEntry(4550),    // 6.6m
            new RangeEntry(4550),    // 6.8m
            new RangeEntry(4550),    // 7.0m 
            new RangeEntry(4550),    // 7.2m 
            new RangeEntry(4550),    // 7.4m 
            new RangeEntry(4550),    // 7.6m 
            new RangeEntry(4550),    // 7.8m 
            new RangeEntry(4550),    // 8.0m 
            new RangeEntry(4550),    // 8.2m 
            new RangeEntry(4550),    // 8.4m 
            new RangeEntry(4550),    // 8.6m 
            new RangeEntry(4550),    // 8.8m 
            new RangeEntry(4550),    // 9.0m 
            new RangeEntry(4550),    // 9.2m 
            new RangeEntry(4550),    // 9.4m
            new RangeEntry(4550),    // 9.6m
            new RangeEntry(4550),    // 9.8m
            new RangeEntry(4550),    // 10.0m
    };

    public RangeTable() {
    }

    /**
     * @param distance in meters
     * @return flywheel speed in RPM from the RangeTable
     */
    public static double get(double distance, boolean isHub) {
        Logger.recordOutput("CustomLogs/RangeTable/InputDistance", distance);
        if(distance <= 0){
            return 0.0;
        }
        if (distance < RANGE_TABLE.MAX_TABLE_DISTANCE) {
            return interpolate(distance, isHub);
        }
        else{
            if (isHub = true){
                return (RANGE_TO_RPM_HUB[RANGE_TO_RPM_HUB.length - 1]).getFlywheelSpeed();
            }
            else {
                return (RANGE_TO_RPM_FLOOR[RANGE_TO_RPM_FLOOR.length - 1]).getFlywheelSpeed();
            }
            
        }
    }

    /**
     * @param distance in meters
     * @return flywheel speed in RPM
     */
    public static double interpolate(double distance, boolean isHub){
        int lowEndEntry = (int)(distance / RANGE_TABLE.RANGE_TABLE_STEP);
        int highEndEntry = lowEndEntry + 1;

        double x1 = lowEndEntry * RANGE_TABLE.RANGE_TABLE_STEP;
        double x2 = highEndEntry * RANGE_TABLE.RANGE_TABLE_STEP;

        double y1 = 0.0;
        double y2 = 0.0;

        if (isHub == true){
            y1 = RANGE_TO_RPM_HUB[lowEndEntry].getFlywheelSpeed();
            y2 = RANGE_TO_RPM_HUB[highEndEntry].getFlywheelSpeed();
        }
        else {
            y1 = RANGE_TO_RPM_FLOOR[lowEndEntry].getFlywheelSpeed();
            y2 = RANGE_TO_RPM_FLOOR[highEndEntry].getFlywheelSpeed();
        }

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