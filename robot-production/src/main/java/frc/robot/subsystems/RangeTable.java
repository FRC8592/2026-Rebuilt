package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.RANGE_TABLE;

public class RangeTable {
    public final static RangeEntry[] RANGE_TO_RPM_HUB = {
            new RangeEntry(2300.0), // 1.2m *
            new RangeEntry(2500.0), // 1.4m *
            new RangeEntry(2750.0), // 1.6m * (03.21)
            new RangeEntry(2950.0), // 1.8m * (03.21)
            new RangeEntry(2950.0), // 2.0m * (03.21)
            new RangeEntry(3200.0), // 2.2m * (03.21)
            new RangeEntry(3300.0), // 2.4m * (03.21)
            new RangeEntry(3650.0), // 2.6m * (03.21) 
            new RangeEntry(3750), // 2.8m * (03.21)
            new RangeEntry(3900), // 3.0m *
            new RangeEntry(4400), // 3.2m * (03.21)
            new RangeEntry(4600.0), // 3.4m * (03.21) 
            new RangeEntry(4725.0), // 3.6m * (03.21)
            new RangeEntry(4950.0), // 3.8m * (03.21)
            new RangeEntry(5100.0), // 4.0m * (03.21)
            new RangeEntry(5300.0), // 4.2m * (03.21)
            new RangeEntry(5450.0), // 4.4m
            new RangeEntry(5500.0), // 4.6m * (03.21)
            new RangeEntry(5700.0), // 4.8m TEST DURING PRACTICE MATCH * (03.21)
    };
    // TODO: Change this later for passing and not be a copy of the hub
    public final static RangeEntry[] RANGE_TO_RPM_FLOOR = {new RangeEntry(4550.0), // 0.0m
            new RangeEntry(4550.0), // 0.2m
            new RangeEntry(4550.0), // 0.4m
            new RangeEntry(4550.0), // 0.6m
            new RangeEntry(4550.0), // 0.8m
            new RangeEntry(4550.0), // 1.0m
            new RangeEntry(4550.0), // 1.2m
            new RangeEntry(4550.0), // 1.4m
            new RangeEntry(4550.0), // 1.6m
            new RangeEntry(4550.0), // 1.8m
            new RangeEntry(4550.0), // 2.0m
            new RangeEntry(4550.0), // 2.2m
            new RangeEntry(4550.0), // 2.4m
            new RangeEntry(4550.0), // 2.6m
            new RangeEntry(4550.0), // 2.8m
            new RangeEntry(4550.0), // 3.0m
            new RangeEntry(4550.0), // 3.2m
            new RangeEntry(4550.0), // 3.4m
            new RangeEntry(4550.0), // 3.6m
            new RangeEntry(4550.0), // 3.8m
            new RangeEntry(4550.0), // 4.0m
            new RangeEntry(4550.0), // 4.2m
            new RangeEntry(4550.0), // 4.4m
            new RangeEntry(4550.0), // 4.6m
            new RangeEntry(4550.0), // 4.8m
            new RangeEntry(4550.0), // 5.0m
            new RangeEntry(4550.0), // 5.2m
            new RangeEntry(4550.0), // 5.4m
            new RangeEntry(4550.0), // 5.6m
            new RangeEntry(4550.0), // 5.8m
            new RangeEntry(4550.0), // 6.0m
            new RangeEntry(4550.0), // 6.2m
            new RangeEntry(4550.0), // 6.4m
            new RangeEntry(4550.0), // 6.6m
            new RangeEntry(4550.0), // 6.8m
            new RangeEntry(4550.0), // 7.0m
            new RangeEntry(4550.0), // 7.2m
            new RangeEntry(4550.0), // 7.4m
            new RangeEntry(4550.0), // 7.6m
            new RangeEntry(4550.0), // 7.8m
            new RangeEntry(4550.0), // 8.0m
            new RangeEntry(4550.0), // 8.2m
            new RangeEntry(4550.0), // 8.4m
            new RangeEntry(4550.0), // 8.6m
            new RangeEntry(4550.0), // 8.8m
            new RangeEntry(4550.0), // 9.0m
            new RangeEntry(4550.0), // 9.2m
            new RangeEntry(4550.0), // 9.4m
            new RangeEntry(4550.0), // 9.6m
            new RangeEntry(4550.0), // 9.8m
            new RangeEntry(4550.0), // 10.0m
    };

    public RangeTable() {}

    /**
     * @param distance in meters
     * @return flywheel speed in RPM from the RangeTable
     */
    public static double get(double distance, boolean isHub) {
        Logger.recordOutput("CustomLogs/RangeTable/InputDistance", distance);
        if (distance <= 0) {
            return 0.0;
        }
        if (distance < RANGE_TABLE.MAX_TABLE_DISTANCE) {
            return interpolate(distance, isHub);
        } else {
            if (isHub = true) {
                return (RANGE_TO_RPM_HUB[RANGE_TO_RPM_HUB.length - 1]).getFlywheelSpeed();
            } else {
                return (RANGE_TO_RPM_FLOOR[RANGE_TO_RPM_FLOOR.length - 1]).getFlywheelSpeed();
            }

        }
    }

    /**
     * @param distance in meters
     * @return flywheel speed in RPM
     */
    public static double interpolate(double distance, boolean isHub) {
        int lowEndEntry = (int) (distance / RANGE_TABLE.RANGE_TABLE_STEP);
        int highEndEntry = lowEndEntry + 1;

        double x1 = lowEndEntry * RANGE_TABLE.RANGE_TABLE_STEP;
        double x2 = highEndEntry * RANGE_TABLE.RANGE_TABLE_STEP;

        double y1 = 0.0;
        double y2 = 0.0;

        if (isHub == true) {
            if(lowEndEntry >= 6 || highEndEntry >= 6){
            y1 = RANGE_TO_RPM_HUB[lowEndEntry - 6].getFlywheelSpeed();
            y2 = RANGE_TO_RPM_HUB[highEndEntry - 6].getFlywheelSpeed();
            }
            else{
                y1 = RANGE_TO_RPM_HUB[0].getFlywheelSpeed();
                y2 = RANGE_TO_RPM_HUB[0].getFlywheelSpeed();
            }
        } else {
            y1 = RANGE_TO_RPM_FLOOR[lowEndEntry].getFlywheelSpeed();
            y2 = RANGE_TO_RPM_FLOOR[highEndEntry].getFlywheelSpeed();
        }

        double interpolationValue = y1 + (distance - x1) * ((y2 - y1) / (x2 - x1));

        return interpolationValue;
    }

    public static class RangeEntry {
        public double flywheelSpeed;

        public RangeEntry(double RPM) {
            flywheelSpeed = RPM;
        }

        public double getFlywheelSpeed() {
            return flywheelSpeed;
        }
    }
}
