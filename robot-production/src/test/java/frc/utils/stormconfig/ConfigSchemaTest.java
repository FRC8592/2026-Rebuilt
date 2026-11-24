package frc.utils.stormconfig;

import static org.junit.jupiter.api.Assertions.*;

import com.typesafe.config.Config;
import com.typesafe.config.ConfigFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;

import frc.utils.stormconfig.ConfigSchema.ConfigSchemaException;
import frc.utils.stormconfig.ConfigSchema.ObjectShape;

import java.io.File;
import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.Test;

class ConfigSchemaTest {

    private static Config parse(String text) {
        return ConfigFactory.parseString(text);
    }

    /** A project made of in-memory files, by name. */
    private static ConfigProject project(Map<String, String> files) {
        return new ConfigProject(name -> {
            String text = files.get(name);
            if (text == null) {
                throw new IllegalArgumentException("no file " + name);
            }
            return parse(text);
        });
    }

    private static final List<String> SIM = List.of("robot.conf", "sim.conf");

    /** {@code base} as config.conf, an empty sim robot, and {@code override} as the sim file. */
    private static ConfigProject simProject(String base, String override) {
        return project(Map.of(
                "config.conf", base + "\nstormconfig { sim = \"robot.conf\", simExtra = \"sim.conf\" }",
                "robot.conf", "",
                "sim.conf", override));
    }

    /** Errors from layering {@code override} (as the sim file) on {@code base}. */
    private static List<String> check(String base, String override) {
        ConfigProject p = simProject(base, override);
        return p.check(p.schema(), SIM);
    }

    private static StormConfig.Node node(String base, String override) {
        ConfigProject p = simProject(base, override);
        return new StormConfig.Node(p.merged(SIM), p.schema(), SIM);
    }

    /** The schema of a project that is just this config.conf. */
    private static ObjectShape schemaOf(String base) {
        return project(Map.of("config.conf", base)).schema();
    }

    /** config.conf plus two robots. */
    private static ConfigProject twoRobots(String base, String robotA, String robotB) {
        return project(Map.of(
                "config.conf", base + "\nstormconfig.robots { \"aa\" = \"a.conf\", \"bb\" = \"b.conf\" }",
                "a.conf", robotA,
                "b.conf", robotB));
    }

    private static void assertOneError(List<String> errors, String containing) {
        assertEquals(1, errors.size(), errors.toString());
        assertTrue(errors.get(0).contains(containing), errors.get(0));
    }

    // ---------------------------------------------------------------- library behavior we rely on

    @Test
    void libraryKeepsNumberTextThroughMergeAndSubstitution() {
        // Typesafe Config parses 180.0 to the same Integer as 180; isDecimalLiteral depends on the
        // original text surviving. If a library upgrade breaks this, this test fails.
        Config c = ConfigFactory.parseString("b = 7.0\nc = 9").withFallback(parse("a = 180\nb = 1\nc = 1.5\nd = 1e3\nx = ${b}")).resolve();
        assertFalse(ConfigTypes.isDecimalLiteral(c.getValue("a")));
        assertTrue(ConfigTypes.isDecimalLiteral(c.getValue("b")));
        assertFalse(ConfigTypes.isDecimalLiteral(c.getValue("c")));
        assertTrue(ConfigTypes.isDecimalLiteral(c.getValue("d")));
        assertTrue(ConfigTypes.isDecimalLiteral(c.getValue("x")));
        assertEquals("7.0", ConfigTypes.numberText(c.getValue("b")));
    }

    // ---------------------------------------------------------------- scalar rules

    @Test
    void doubleNeedsDecimalPoint() {
        assertOneError(check("kP = 1.5", "kP = 52"), "write 52.0");
        assertEquals(List.of(), check("kP = 1.5", "kP = 52.0"));
    }

    @Test
    void intRejectsDecimalPoint() {
        assertOneError(check("canId = 1", "canId = 20.0"), "can't have a decimal point");
    }

    @Test
    void wholeNumberDoubleStaysDouble() {
        Object value = node("gear = 80.0", "").get("gear");
        assertEquals(Double.class, value.getClass());
    }

    @Test
    void stringCantReplaceNumber() {
        assertOneError(check("kP = 1.5", "kP = \"1.5\""), "should be double");
    }

    @Test
    void simFileTypoIsAnError() {
        assertOneError(check("SWERVE { STEER_KP = 1.0 }", "SWERVE.STEER_KPP = 2.0"), "can only override");
    }

    @Test
    void defaultsApplyWhenRobotDoesNotOverride() {
        assertEquals(List.of(), check("canId = 0\nkP = 0.0", "kP = 1.5"));
        assertEquals(0, (Integer) node("canId = 0\nkP = 0.0", "kP = 1.5").get("canId"));
    }

    // ---------------------------------------------------------------- the union of robot files

    @Test
    void robotOnlyKeyNeedNotBeInEveryRobot() {
        ConfigProject p = twoRobots("TURRET.LOG_PATH = \"Turret/\"", "TURRET.CAN_ID = 20", "");
        ObjectShape schema = p.schema();
        assertEquals(List.of(), p.checkAll(schema));

        StormConfig.Node a = new StormConfig.Node(p.merged(List.of("a.conf")), schema, List.of("a.conf"));
        a.require("TURRET", "Constants.TURRET");
        assertEquals(20, (Integer) a.get("TURRET.CAN_ID"));
    }

    @Test
    void usingAnUnconfiguredSectionFailsClearly() {
        ConfigProject p = twoRobots("TURRET.LOG_PATH = \"Turret/\"", "TURRET { CAN_ID = 20, GEAR = 80.0 }", "");
        StormConfig.Node b = new StormConfig.Node(p.merged(List.of("b.conf")), p.schema(), List.of("b.conf"));
        StormConfig.MissingConfigException e = assertThrows(StormConfig.MissingConfigException.class,
                () -> b.require("TURRET", "Constants.TURRET"));
        assertTrue(e.getMessage().contains("Constants.TURRET was used"), e.getMessage());
        assertTrue(e.getMessage().contains("TURRET.CAN_ID, TURRET.GEAR"), e.getMessage());
        assertTrue(e.getMessage().contains("Set them in b.conf"), e.getMessage());
    }

    @Test
    void keyHasOneTypeAcrossRobots() {
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class,
                () -> twoRobots("", "TURRET.GEAR = 80.0", "TURRET.GEAR = 80").schema());
        assertTrue(e.getMessage().contains("write 80.0"), e.getMessage());
    }

    @Test
    void nearMissKeysAreTypos() {
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class,
                () -> twoRobots("SWERVE.SNAP_TO_kP = 1.0", "SWERVE.SNAP_TO_KP = 2.0", "").schema());
        assertTrue(e.getMessage().contains("looks like a typo of SWERVE.SNAP_TO_kP"), e.getMessage());
    }

    @Test
    void simFileCantAddKeysEvenIfAnotherRobotHasThem() {
        ConfigProject p = project(Map.of(
                "config.conf", "toggles.useDrive = true\nstormconfig { robots { \"aa\" = \"a.conf\" }, sim = \"b.conf\", simExtra = \"sim.conf\" }",
                "a.conf", "TURRET.CAN_ID = 20",
                "b.conf", "",
                "sim.conf", "TURRET.CAN_ID = 5"));
        assertOneError(p.check(p.schema(), List.of("b.conf", "sim.conf")), "sim files can only override");
    }

    // ---------------------------------------------------------------- measures

    @Test
    void measureKeepsItsDimension() {
        assertOneError(check("offset = \"286 deg\"", "offset = \"1.5 m\""), "should be Angle");
        assertEquals(List.of(), check("offset = \"286 deg\"", "offset = \"5 rad\""));
    }

    @Test
    void measureDecodesToWpilibUnits() {
        Angle angle = node("offset = \"180 deg\"", "").get("offset");
        assertEquals(Math.PI, angle.baseUnitMagnitude(), 1e-9);
    }

    @Test
    void unknownUnitIsAnError() {
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class, () -> schemaOf("d = \"5 meters\""));
        assertTrue(e.getMessage().contains("unknown unit 'meters'"), e.getMessage());
    }

    // ---------------------------------------------------------------- compound types

    @Test
    void compoundAnglesNeedUnits() {
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class,
                () -> schemaOf("p = { _type = Pose2d, x = 1.0, y = 2.0, rotation = 90 }"));
        assertTrue(e.getMessage().contains("needs a unit"), e.getMessage());
    }

    @Test
    void compoundOverrideMergesFieldByField() {
        String base = "cam = { _type = Transform3d, x = 1.0, y = 2.0, z = 0.5, yaw = \"90 deg\" }";
        Transform3d t = node(base, "cam.yaw = \"180 deg\"").get("cam");
        assertEquals(1.0, t.getX(), 1e-9);
        assertEquals(Math.PI, t.getRotation().getZ(), 1e-9);
    }

    @Test
    void compoundTypeCantChange() {
        assertOneError(check("p = { _type = Pose2d, x = 1.0, y = 2.0 }", "p = { _type = Translation2d, x = 1.0, y = 2.0 }"),
                "should be Pose2d");
    }

    @Test
    void compoundUnknownFieldIsAnError() {
        List<String> errors = check("p = { _type = Pose2d, x = 1.0, y = 2.0 }", "p.z = 3.0");
        assertFalse(errors.isEmpty());
        assertTrue(errors.get(0).contains("unknown field 'z'"), errors.toString());
    }

    @Test
    void colorChannelsAreChecked() {
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class,
                () -> schemaOf("c = { _type = Color, r = 300, g = 0, b = 0 }"));
        assertTrue(e.getMessage().contains("0 to 255"), e.getMessage());
    }

    // ---------------------------------------------------------------- substitutions and lists

    @Test
    void substitutionUsesOverriddenValue() {
        String base = "SHARED.LOG_FOLDER = \"CustomLogs\"\nVISION.LOG_PATH = ${SHARED.LOG_FOLDER}\"/Vision/\"";
        String path = node(base, "SHARED.LOG_FOLDER = \"SimLogs\"").get("VISION.LOG_PATH");
        assertEquals("SimLogs/Vision/", path);
    }

    @Test
    void listElementsMustMatch() {
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class, () -> schemaOf("ids = [1, 2.5]"));
        assertTrue(e.getMessage().contains("same type"), e.getMessage());
    }

    @Test
    void listOfObjectsReadsAsRecords() {
        record Camera(String name, Transform3d offset) {}
        String base = "cams = [ { name = \"left\", offset = { _type = Transform3d, x = 0.0, y = 0.3, z = 0.5 } } ]";
        String override = "cams = [ { name = \"a\", offset = { _type = Transform3d, x = 0.0, y = 0.0, z = 0.0 } },"
                + " { name = \"b\", offset = { _type = Transform3d, x = 1.0, y = 0.0, z = 0.0 } } ]";
        List<Camera> cams = node(base, override).getList("cams", n -> new Camera(n.get("name"), n.get("offset")));
        assertEquals(2, cams.size());
        assertEquals("b", cams.get(1).name());
        assertEquals(1.0, cams.get(1).offset().getX(), 1e-9);
        assertOneError(check(base, "cams = [ { name = \"a\" } ]"), "missing");
    }

    // ---------------------------------------------------------------- the real files

    // ---------------------------------------------------------------- dashboard tunables

    // One key per line: schema order is line order
    private static final String TUNABLE_BASE = "T {\nP = \"40 V\"\nD = 0.8\nON = true\nPOSE { _type = Pose2d, x = 1.0, y = 2.0 }\n}\n";

    @Test
    void tunablesListIsNotAValueAndComesOutInSchemaOrder() {
        ConfigProject p = twoRobots(TUNABLE_BASE + "T._tunables = [\"ON\", \"P\"]", "", "");
        ObjectShape schema = p.schema();
        assertFalse(schema.fields().get("T") instanceof ObjectShape t && t.fields().containsKey("_tunables"));
        assertEquals(List.of(), p.checkAll(schema));
        assertEquals(Map.of("T", List.of("P", "ON")), p.tunables(schema));
    }

    @Test
    void robotFilesAddTunablesToTheirSections() {
        ConfigProject p = twoRobots(TUNABLE_BASE + "T._tunables = [\"P\"]",
                "T { X = 1, _tunables = [\"X\"] }", "T._tunables = [\"D\"]");
        assertEquals(Map.of("T", List.of("P", "D", "X")), p.tunables(p.schema()));
    }

    @Test
    void tunableMistakesAreReported() {
        ConfigProject p = twoRobots(TUNABLE_BASE + "T._tunables = [\"Q\", \"POSE\"]\n_tunables = [\"T\"]", "", "");
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class, () -> p.tunables(p.schema()));
        assertTrue(e.getMessage().contains("T has no value Q"), e.getMessage());
        assertTrue(e.getMessage().contains("POSE is Pose2d"), e.getMessage());
        assertTrue(e.getMessage().contains("goes inside a section"), e.getMessage());
    }

    @Test
    void simFileCanOnlyRepeatTunables() {
        ConfigProject ok = simProject(TUNABLE_BASE + "T._tunables = [\"P\"]", "T._tunables = [\"P\"]");
        assertEquals(Map.of("T", List.of("P")), ok.tunables(ok.schema()));
        ConfigProject bad = simProject(TUNABLE_BASE, "T._tunables = [\"D\"]");
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class, () -> bad.tunables(bad.schema()));
        assertTrue(e.getMessage().contains("sim files can't add tunables"), e.getMessage());
        assertEquals(List.of(), bad.check(bad.schema(), SIM));
    }

    @Test
    void misspelledReservedKeyIsAnError() {
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class,
                () -> schemaOf(TUNABLE_BASE + "T._tunable = [\"P\"]"));
        assertTrue(e.getMessage().contains("T._tunable: keys starting with _ are reserved"), e.getMessage());
    }

    @Test
    void measureTunablesShowInTheirConfigUnit() {
        assertEquals("deg", ConfigTypes.unitOf("15 deg"));
        assertNull(ConfigTypes.unitOf("hello"));
        Angle angle = (Angle) ConfigTypes.measure(15.0, "deg");
        assertEquals(15.0, ConfigTypes.in(angle, "deg"), 1e-12);
        assertEquals(Math.toRadians(15.0), ConfigTypes.in(angle, "rad"), 1e-12);
    }

    @Test
    void deployedConfigFilesPass() {
        File dir = new File("src/main/deploy");
        ConfigProject p = new ConfigProject(name -> ConfigFactory.parseFile(new File(dir, name)));
        ObjectShape schema = p.schema();
        assertEquals(List.of(), p.checkAll(schema));
        assertEquals(List.of("TURRET_P", "TURRET_I", "TURRET_D", "TURRET_S", "TURRET_V", "TURRET_A"), p.tunables(schema).get("TURRET"));

        // Prismarine sets every section, so no section of Constants would fail on it
        List<String> sim = List.of("prismarine.conf", "prismarine.sim.conf");
        StormConfig.Node node = new StormConfig.Node(p.merged(sim), schema, sim);
        requireAll(node, schema, "");

        Pose2d hub = node.get("SCORING.RED_HUB_POSE");
        assertEquals(11.94, hub.getX(), 1e-9);
        Color pink = node.get("LEDS.HOT_PINK");
        assertEquals(29 / 255.0, pink.green, 1e-3); // Color stores channels at reduced precision
        assertEquals(80.0, (Double) node.get("TURRET.TURRET_GT"), 1e-9);
        assertEquals("CustomLogs/TURRET/", node.get("TURRET.LOG_PATH"));
    }

    private static void requireAll(StormConfig.Node node, ObjectShape shape, String path) {
        node.require(path, path.isEmpty() ? "Constants" : path);
        shape.fields().forEach((key, field) -> {
            if (field instanceof ObjectShape obj) {
                requireAll(node, obj, path.isEmpty() ? key : path + "." + key);
            }
        });
    }
}
