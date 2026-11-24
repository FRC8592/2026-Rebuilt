package frc.utils.stormconfig;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A config value that can be changed from the dashboard while the robot runs. Code normally reads it
 * through the generated accessor, e.g. {@code Constants.TURRET.TURRET_P()}, and reacts to changes
 * with the section's {@code onTune}; this object is for {@link #set} or for passing as a Supplier.
 *
 * <p>The value lives here: {@link #get} never touches NetworkTables. {@link TunableBackend#update}
 * syncs it with the dashboard once per loop. The API matches WPILib 2027's tunables (get, set,
 * Supplier, Consumer) so that moving onto them only changes the backend.
 */
public final class Tunable<T> implements Supplier<T>, Consumer<T> {
    private final String path;
    private final TunableSection section;
    private T value;
    private boolean setInCode;

    Tunable(String path, T initialValue, TunableSection section) {
        this.path = path;
        this.value = Objects.requireNonNull(initialValue);
        this.section = section;
    }

    /** The current value. */
    @Override
    public T get() {
        return value;
    }

    /** Changes the value from code (e.g. resetting a dashboard button); the dashboard shows it next loop. */
    public void set(T newValue) {
        if (!Objects.equals(Objects.requireNonNull(newValue), value)) {
            value = newValue;
            setInCode = true;
        }
    }

    @Override
    public void accept(T newValue) {
        set(newValue);
    }

    /** The config path, e.g. "TURRET.TURRET_P". */
    public String path() {
        return path;
    }

    /** A change from the dashboard. Doesn't count as set in code, so it isn't published back. */
    void tune(T newValue) {
        if (newValue != null && !Objects.equals(newValue, value)) {
            value = newValue;
            section.markTuned();
            System.out.println("Tunable " + path + " tuned to " + value);
        }
    }

    /** Whether code called set() since the last call to this; the backend then publishes the value. */
    boolean takeSetInCode() {
        boolean result = setInCode;
        setInCode = false;
        return result;
    }

    @Override
    public String toString() {
        return path + " = " + value;
    }
}
