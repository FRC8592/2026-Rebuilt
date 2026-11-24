package frc.utils.stormconfig;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

/**
 * The tunables of one config section (e.g. TURRET), and the callbacks to run when the dashboard
 * changes any of them. Generated code exposes this as {@code Constants.TURRET.onTune(...)}.
 */
public final class TunableSection {
    private final String path;
    private final List<Runnable> callbacks = new ArrayList<>();
    private boolean tuned;

    TunableSection(String path) {
        this.path = path;
    }

    /**
     * Runs {@code callback} on the robot thread, once per loop in which the dashboard changed any
     * tunable in this section. Not run for changes made by code with set().
     */
    public synchronized void onTune(Runnable callback) {
        callbacks.add(callback);
    }

    void markTuned() {
        tuned = true;
    }

    /** Runs the callbacks if anything in this section was tuned since the last call. */
    void runCallbacks() {
        if (!tuned) {
            return;
        }
        tuned = false;
        List<Runnable> toRun;
        synchronized (this) {
            toRun = List.copyOf(callbacks);
        }
        for (Runnable callback : toRun) {
            try {
                callback.run();
            } catch (RuntimeException e) {
                // A bad callback shouldn't stop the robot; report it and keep going
                DriverStation.reportError("onTune callback for " + path + " failed: " + e, e.getStackTrace());
            }
        }
    }
}
