package frc.utils.stormconfig;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.*;

import com.typesafe.config.ConfigFactory;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;

import java.util.List;

import org.junit.jupiter.api.Test;

class TunableTest {

    /** A node over a one-file project; each test uses its own section so the shared backend doesn't mix them. */
    private static StormConfig.Node node(String base) {
        ConfigProject p = new ConfigProject(name -> {
            if (!name.equals("config.conf")) {
                throw new IllegalArgumentException("no file " + name);
            }
            return ConfigFactory.parseString(base);
        });
        return new StormConfig.Node(p.merged(List.of()), p.schema(), List.of());
    }

    /** Stands in for the dashboard: another publisher on the tunable's topic. */
    private static DoublePublisher dashboard(String table, String key) {
        return NetworkTableInstance.getDefault().getTable("Tunables/" + table).getDoubleTopic(key).publish();
    }

    @Test
    void dashboardChangesReachTheValueAndRunOnTuneOnce() {
        StormConfig.Node n = node("A { P = 1.5, Q = 2.5, _tunables = [\"P\", \"Q\"] }");
        Tunable<Double> p = n.tunable("A.P");
        Tunable<Double> q = n.tunable("A.Q");
        int[] calls = {0};
        n.section("A").onTune(() -> calls[0]++);

        TunableBackend.update();
        assertEquals(0, calls[0], "publishing the config value isn't a tune");

        dashboard("A", "P").set(3.0);
        dashboard("A", "Q").set(4.0);
        TunableBackend.update();
        assertEquals(3.0, p.get());
        assertEquals(4.0, q.get());
        assertEquals(1, calls[0], "one call per loop, however many values changed");

        TunableBackend.update();
        assertEquals(1, calls[0]);
    }

    @Test
    void setInCodeIsPublishedButIsNotATune() {
        StormConfig.Node n = node("B { ON = true, _tunables = [\"ON\"] }");
        Tunable<Boolean> on = n.tunable("B.ON");
        int[] calls = {0};
        n.section("B").onTune(() -> calls[0]++);

        on.set(false);
        TunableBackend.update();
        assertFalse(NetworkTableInstance.getDefault().getTable("Tunables/B").getBooleanTopic("ON").subscribe(true).get());
        assertEquals(0, calls[0]);
    }

    @Test
    void unitValuesShowInTheConfigUnit() {
        StormConfig.Node n = node("C { ANGLE = \"15 deg\", _tunables = [\"ANGLE\"] }");
        Tunable<Angle> angle = n.tunable("C.ANGLE");
        TunableBackend.update();
        assertEquals(15.0, NetworkTableInstance.getDefault().getTable("Tunables/C").getDoubleTopic("ANGLE").subscribe(0).get(), 1e-9);

        dashboard("C", "ANGLE").set(30.0);
        TunableBackend.update();
        assertEquals(30.0, angle.get().in(Degrees), 1e-9);
    }

    @Test
    void aFailingCallbackDoesNotStopTheOthers() {
        StormConfig.Node n = node("D { X = 1, _tunables = [\"X\"] }");
        Tunable<Integer> x = n.tunable("D.X");
        int[] calls = {0};
        n.section("D").onTune(() -> {
            throw new IllegalStateException("boom");
        });
        n.section("D").onTune(() -> calls[0]++);

        dashboard("D", "X").set(7.0);
        TunableBackend.update();
        assertEquals(7, x.get());
        assertEquals(1, calls[0]);
    }
}
