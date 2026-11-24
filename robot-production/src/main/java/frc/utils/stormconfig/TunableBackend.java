package frc.utils.stormconfig;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.Topic;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleFunction;
import java.util.function.ToDoubleFunction;

/**
 * Connects {@link Tunable}s to the dashboard through NetworkTables, under "Tunables/SECTION/KEY".
 * This is the only class that knows how tunables are published: on WPILib 2027 it becomes a thin
 * layer over {@code org.wpilib.tunable} (publishDouble/publishBoolean with a getter and setter,
 * and an onTune hook that marks the section), and Robot no longer needs to call {@link #update}.
 */
public final class TunableBackend {
    private static final String ROOT = "Tunables";

    private static final List<Runnable> bindings = new ArrayList<>();
    private static final Map<String, TunableSection> sections = new LinkedHashMap<>();

    private TunableBackend() {}

    /**
     * Picks up dashboard changes, publishes values changed by code, then runs the onTune callbacks
     * of every section that changed. Call once per loop from Robot.robotPeriodic.
     */
    public static void update() {
        List<Runnable> toUpdate;
        List<TunableSection> toNotify;
        synchronized (TunableBackend.class) {
            toUpdate = List.copyOf(bindings);
            toNotify = List.copyOf(sections.values());
        }
        toUpdate.forEach(Runnable::run);
        // Callbacks run after every value is updated, so they see all of this loop's changes
        toNotify.forEach(TunableSection::runCallbacks);
    }

    static synchronized TunableSection section(String path) {
        return sections.computeIfAbsent(path, TunableSection::new);
    }

    /** A number on the dashboard; {@code unit} (may be null) is shown as a topic property. */
    static <T> void publishDouble(Tunable<T> tunable, ToDoubleFunction<T> toNumber, DoubleFunction<T> fromNumber,
            String unit) {
        double initial = toNumber.applyAsDouble(tunable.get());
        DoubleEntry entry = table(tunable).getDoubleTopic(key(tunable)).getEntry(initial);
        describe(entry.getTopic(), unit);
        entry.set(initial);
        double[] last = {initial};
        add(() -> {
            // entry.get() reads NetworkTables' local copy; nothing goes over the network unless a value changed
            double remote = entry.get();
            if (Double.compare(remote, last[0]) != 0) {
                last[0] = remote;
                tunable.tune(fromNumber.apply(remote));
            }
            if (tunable.takeSetInCode()) {
                last[0] = toNumber.applyAsDouble(tunable.get());
                entry.set(last[0]);
            }
        });
    }

    static void publishBoolean(Tunable<Boolean> tunable) {
        boolean initial = tunable.get();
        BooleanEntry entry = table(tunable).getBooleanTopic(key(tunable)).getEntry(initial);
        describe(entry.getTopic(), null);
        entry.set(initial);
        boolean[] last = {initial};
        add(() -> {
            boolean remote = entry.get();
            if (remote != last[0]) {
                last[0] = remote;
                tunable.tune(remote);
            }
            if (tunable.takeSetInCode()) {
                last[0] = tunable.get();
                entry.set(last[0]);
            }
        });
    }

    static void publishString(Tunable<String> tunable) {
        String initial = tunable.get();
        StringEntry entry = table(tunable).getStringTopic(key(tunable)).getEntry(initial);
        describe(entry.getTopic(), null);
        entry.set(initial);
        String[] last = {initial};
        add(() -> {
            String remote = entry.get();
            if (!remote.equals(last[0])) {
                last[0] = remote;
                tunable.tune(remote);
            }
            if (tunable.takeSetInCode()) {
                last[0] = tunable.get();
                entry.set(last[0]);
            }
        });
    }

    private static synchronized void add(Runnable binding) {
        bindings.add(binding);
    }

    /** "Tunables/TURRET" for TURRET.TURRET_P. */
    private static NetworkTable table(Tunable<?> tunable) {
        String path = tunable.path();
        String section = path.substring(0, path.lastIndexOf('.'));
        return NetworkTableInstance.getDefault().getTable(ROOT + "/" + section.replace('.', '/'));
    }

    private static String key(Tunable<?> tunable) {
        return tunable.path().substring(tunable.path().lastIndexOf('.') + 1);
    }

    private static void describe(Topic topic, String unit) {
        if (unit != null) {
            topic.setProperty("unit", "\"" + unit + "\"");
        }
    }
}
