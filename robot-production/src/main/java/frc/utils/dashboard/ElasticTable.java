package frc.utils.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A named NetworkTables table under "Elastic/". Base class for anything that
 * publishes to or reads from the Elastic dashboard (options, tuners, etc.).
 */
public class ElasticTable {

    protected final NetworkTable table;
    private final String name;

    public ElasticTable(String tableName) {
        this.name = tableName;
        this.table = NetworkTableInstance.getDefault().getTable("Elastic/" + tableName);
    }

    public String getName() {
        return name;
    }

    public NetworkTable getTable() {
        return table;
    }
}
