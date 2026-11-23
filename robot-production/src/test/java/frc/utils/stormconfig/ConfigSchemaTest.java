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

import org.junit.jupiter.api.Test;

class ConfigSchemaTest {

    private static Config parse(String text) {
        return ConfigFactory.parseString(text);
    }

    /** Errors from checking {@code override} layered on {@code base}. */
    private static List<String> check(String base, String override) {
        Config b = parse(base);
        ObjectShape schema = ConfigSchema.fromBase(b.resolve());
        return ConfigSchema.check(schema, ConfigSchema.merge(b, List.of(parse(override))));
    }

    private static StormConfig.Node node(String base, String override) {
        Config b = parse(base);
        return new StormConfig.Node(ConfigSchema.merge(b, List.of(parse(override))), ConfigSchema.fromBase(b.resolve()));
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
    void overrideTypoIsAnError() {
        assertOneError(check("SWERVE { STEER_KP = 1.0 }", "SWERVE.STEER_KPP = 2.0"), "not in config.conf");
    }

    // ---------------------------------------------------------------- placeholders

    @Test
    void placeholderMustBeSet() {
        assertOneError(check("canId = \"@int\"", ""), "no value set");
        assertEquals(List.of(), check("canId = \"@int\"", "canId = 20"));
        assertOneError(check("canId = \"@int\"", "canId = 20.0"), "should be int");
    }

    @Test
    void unknownPlaceholderIsAnError() {
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class, () -> ConfigSchema.fromBase(parse("a = \"@integer\"")));
        assertTrue(e.getMessage().contains("unknown placeholder"), e.getMessage());
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
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class, () -> ConfigSchema.fromBase(parse("d = \"5 meters\"")));
        assertTrue(e.getMessage().contains("unknown unit 'meters'"), e.getMessage());
    }

    // ---------------------------------------------------------------- compound types

    @Test
    void compoundAnglesNeedUnits() {
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class,
                () -> ConfigSchema.fromBase(parse("p = { _type = Pose2d, x = 1.0, y = 2.0, rotation = 90 }")));
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
                () -> ConfigSchema.fromBase(parse("c = { _type = Color, r = 300, g = 0, b = 0 }")));
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
        ConfigSchemaException e = assertThrows(ConfigSchemaException.class, () -> ConfigSchema.fromBase(parse("ids = [1, 2.5]")));
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

    @Test
    void deployedConfigFilesPass() {
        File dir = new File("src/main/deploy");
        Config base = ConfigFactory.parseFile(new File(dir, "config.conf"));
        ObjectShape schema = ConfigSchema.fromBase(base.resolve());
        Config sim = ConfigSchema.merge(base, List.of(
                ConfigFactory.parseFile(new File(dir, "prismarine.conf")),
                ConfigFactory.parseFile(new File(dir, "prismarine.sim.conf"))));
        assertEquals(List.of(), ConfigSchema.check(schema, sim));

        StormConfig.Node node = new StormConfig.Node(sim, schema);
        Pose2d hub = node.get("SCORING.RED_HUB_POSE");
        assertEquals(11.94, hub.getX(), 1e-9);
        Color pink = node.get("LEDS.HOT_PINK");
        assertEquals(29 / 255.0, pink.green, 1e-3); // Color stores channels at reduced precision
        assertEquals(80.0, (Double) node.get("TURRET.TURRET_GT"), 1e-9);
        assertEquals("CustomLogs/TURRET/", node.get("TURRET.LOG_PATH"));
    }
}
