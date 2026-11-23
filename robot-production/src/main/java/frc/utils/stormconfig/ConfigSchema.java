package frc.utils.stormconfig;

import com.typesafe.config.Config;
import com.typesafe.config.ConfigList;
import com.typesafe.config.ConfigObject;
import com.typesafe.config.ConfigValue;
import com.typesafe.config.ConfigValueType;

import frc.utils.stormconfig.ConfigTypes.CompoundType;
import frc.utils.stormconfig.ConfigTypes.MeasureType;
import frc.utils.stormconfig.ConfigTypes.ParsedMeasure;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The shape of the config, taken from the base file (config.conf), and the rules every robot's
 * merged config must follow:
 * <ul>
 *   <li>config.conf defines every key with a default;
 *   <li>a value's type is whatever the base file's value is: 42 is an int, 42.0 is a double,
 *       "-15 deg" is an Angle, {@code { _type = Pose2d, ... }} is a Pose2d;
 *   <li>robot and sim files only override: they may change values but not types, and may not add
 *       keys (so typos fail).
 * </ul>
 *
 * <p>Compiled into both the robot code and the build-time generator.
 */
public final class ConfigSchema {
    /** Top-level block holding robot selection settings; not part of the generated constants. */
    public static final String META_KEY = "stormconfig";

    public sealed interface Shape permits Scalar, MeasureShape, CompoundShape, ObjectShape, ListShape {
        /** How this type is described in error messages. */
        String describe();
    }

    public enum ScalarKind { BOOLEAN, INT, LONG, DOUBLE, STRING }

    public record Scalar(ScalarKind kind) implements Shape {
        public String describe() {
            return kind == ScalarKind.STRING ? "String" : kind.name().toLowerCase();
        }
    }

    public record MeasureShape(MeasureType type) implements Shape {
        public String describe() {
            return type.javaType();
        }
    }

    public record CompoundShape(CompoundType type) implements Shape {
        public String describe() {
            return type.name();
        }
    }

    /** A plain object. Fields keep the order they appear in the base file. */
    public record ObjectShape(Map<String, Shape> fields) implements Shape {
        public String describe() {
            return "object";
        }

        /** The shape at a dotted path below this object. */
        public Shape lookup(List<String> path) {
            Shape shape = this;
            for (String key : path) {
                shape = ((ObjectShape) shape).fields().get(key);
                if (shape == null) {
                    throw new IllegalArgumentException("no config key " + String.join(".", path));
                }
            }
            return shape;
        }
    }

    public record ListShape(Shape element) implements Shape {
        public String describe() {
            return "list of " + element.describe();
        }
    }

    /** Thrown when a config file breaks the rules; the message lists every problem found. */
    public static final class ConfigSchemaException extends RuntimeException {
        public ConfigSchemaException(String what, List<String> errors) {
            super(what + " has " + errors.size() + " config error(s):\n  " + String.join("\n  ", errors));
        }
    }

    private ConfigSchema() {}

    // ---------------------------------------------------------------- building the schema

    /** Reads the shape of the resolved base config. */
    public static ObjectShape fromBase(Config resolvedBase) {
        List<String> errors = new ArrayList<>();
        ObjectShape shape = objectShape(resolvedBase.root(), "", errors);
        if (!errors.isEmpty()) {
            throw new ConfigSchemaException(where(resolvedBase.root()), errors);
        }
        return shape;
    }

    private static Shape shapeOf(ConfigValue value, String path, List<String> errors) {
        switch (value.valueType()) {
            case BOOLEAN:
                return new Scalar(ScalarKind.BOOLEAN);
            case NUMBER:
                if (ConfigTypes.isDecimalLiteral(value)) {
                    return new Scalar(ScalarKind.DOUBLE);
                }
                return new Scalar(value.unwrapped() instanceof Long ? ScalarKind.LONG : ScalarKind.INT);
            case STRING:
                return stringShape(value, path, errors);
            case OBJECT:
                ConfigObject obj = (ConfigObject) value;
                if (obj.containsKey(ConfigTypes.TYPE_KEY)) {
                    return compoundShape(obj, path, errors);
                }
                return objectShape(obj, path, errors);
            case LIST:
                return listShape((ConfigList) value, path, errors);
            default:
                errors.add(where(value) + " " + path + ": null is not allowed; give a default value");
                return null;
        }
    }

    private static Shape stringShape(ConfigValue value, String path, List<String> errors) {
        String text = (String) value.unwrapped();
        try {
            ParsedMeasure measure = ConfigTypes.parseMeasure(text);
            return measure == null ? new Scalar(ScalarKind.STRING) : new MeasureShape(measure.type());
        } catch (IllegalArgumentException e) {
            errors.add(where(value) + " " + path + ": " + e.getMessage());
            return null;
        }
    }

    private static Shape compoundShape(ConfigObject obj, String path, List<String> errors) {
        try {
            CompoundType type = ConfigTypes.compoundType(obj);
            List<String> fieldErrors = new ArrayList<>();
            ConfigTypes.compoundFields(type, obj, fieldErrors);
            fieldErrors.forEach(e -> errors.add(where(obj) + " " + path + ": " + e));
            return new CompoundShape(type);
        } catch (IllegalArgumentException e) {
            errors.add(where(obj) + " " + path + ": " + e.getMessage());
            return null;
        }
    }

    private static ObjectShape objectShape(ConfigObject obj, String path, List<String> errors) {
        Map<String, Shape> fields = new LinkedHashMap<>();
        for (String key : keysInFileOrder(obj)) {
            if (path.isEmpty() && key.equals(META_KEY)) {
                continue;
            }
            Shape shape = shapeOf(obj.get(key), join(path, key), errors);
            if (shape != null) {
                fields.put(key, shape);
            }
        }
        return new ObjectShape(fields);
    }

    private static Shape listShape(ConfigList list, String path, List<String> errors) {
        if (list.isEmpty()) {
            errors.add(where(list) + " " + path + ": the base file needs at least one element so the element type is known");
            return null;
        }
        Shape element = shapeOf(list.get(0), path + "[0]", errors);
        for (int i = 1; i < list.size() && element != null; i++) {
            Shape other = shapeOf(list.get(i), path + "[" + i + "]", errors);
            if (other != null && !other.equals(element)) {
                errors.add(where(list.get(i)) + " " + path + "[" + i + "]: every element must have the same type; element 0 is "
                        + element.describe() + " but this is " + other.describe());
            }
        }
        return element == null ? null : new ListShape(element);
    }

    // ---------------------------------------------------------------- checking a merged config

    /** Checks a resolved, merged config (base + robot + sim) against the schema. Returns every problem. */
    public static List<String> check(ObjectShape schema, Config resolvedMerged) {
        List<String> errors = new ArrayList<>();
        checkObject(schema, resolvedMerged.root(), "", true, errors);
        return errors;
    }

    private static void check(Shape expected, ConfigValue value, String path, List<String> errors) {
        if (expected instanceof ObjectShape obj) {
            if (value.valueType() != ConfigValueType.OBJECT || ((ConfigObject) value).containsKey(ConfigTypes.TYPE_KEY)) {
                errors.add(mismatch(path, expected, value));
            } else {
                checkObject(obj, (ConfigObject) value, path, false, errors);
            }
            return;
        }
        if (expected instanceof ListShape list) {
            if (value.valueType() != ConfigValueType.LIST) {
                errors.add(mismatch(path, expected, value));
                return;
            }
            ConfigList values = (ConfigList) value;
            for (int i = 0; i < values.size(); i++) {
                check(list.element(), values.get(i), path + "[" + i + "]", errors);
            }
            return;
        }
        List<String> leafErrors = new ArrayList<>();
        Shape actual = shapeOf(value, path, leafErrors);
        errors.addAll(leafErrors);
        if (actual != null && !actual.equals(expected)) {
            errors.add(mismatch(path, expected, value));
        }
    }

    private static void checkObject(ObjectShape expected, ConfigObject obj, String path, boolean root, List<String> errors) {
        for (String key : obj.keySet()) {
            if (root && key.equals(META_KEY)) {
                continue;
            }
            if (!expected.fields().containsKey(key)) {
                errors.add(where(obj.get(key)) + " " + join(path, key)
                        + ": not in config.conf; robot and sim files can only override existing keys (a typo? new keys go in config.conf first)");
            }
        }
        for (Map.Entry<String, Shape> field : expected.fields().entrySet()) {
            ConfigValue value = obj.get(field.getKey());
            if (value == null) {
                errors.add(where(obj) + " " + join(path, field.getKey()) + ": missing");
            } else {
                check(field.getValue(), value, join(path, field.getKey()), errors);
            }
        }
    }

    private static String mismatch(String path, Shape expected, ConfigValue value) {
        String hint = "";
        String shown = value.render();
        if (value.valueType() == ConfigValueType.NUMBER) {
            shown = ConfigTypes.numberText(value);
            boolean decimal = ConfigTypes.isDecimalLiteral(value);
            if (expected instanceof Scalar s && s.kind() == ScalarKind.DOUBLE && !decimal) {
                hint = " (write " + shown + ".0)";
            } else if (expected instanceof Scalar s && s.kind() == ScalarKind.INT && decimal) {
                hint = " (int values can't have a decimal point)";
            }
        }
        return where(value) + " " + path + ": should be " + expected.describe() + " but is " + shown + hint;
    }

    // ---------------------------------------------------------------- reading values

    /** Converts a checked config value to the Java value the generated code expects. */
    public static Object decode(Shape shape, ConfigValue value) {
        if (shape instanceof Scalar s) {
            Object v = value.unwrapped();
            switch (s.kind()) {
                case INT:
                    return ((Number) v).intValue();
                case LONG:
                    return ((Number) v).longValue();
                case DOUBLE:
                    return ((Number) v).doubleValue();
                default:
                    return v;
            }
        }
        if (shape instanceof MeasureShape) {
            return ConfigTypes.parseMeasure((String) value.unwrapped()).value();
        }
        if (shape instanceof CompoundShape c) {
            List<String> errors = new ArrayList<>();
            Map<String, Double> fields = ConfigTypes.compoundFields(c.type(), (ConfigObject) value, errors);
            if (!errors.isEmpty()) {
                throw new ConfigSchemaException(where(value), errors);
            }
            return c.type().build().apply(fields);
        }
        if (shape instanceof ListShape l) {
            List<Object> result = new ArrayList<>();
            for (ConfigValue element : (ConfigList) value) {
                result.add(decode(l.element(), element));
            }
            return List.copyOf(result);
        }
        throw new IllegalArgumentException("objects are read through StormConfig.Node, not decoded directly");
    }

    // ---------------------------------------------------------------- loading

    /** The robot-selection settings in the {@code stormconfig} block of config.conf. */
    public record Meta(Map<String, String> robots, String sim, String simExtra, boolean debug) {}

    public static Meta meta(Config base) {
        Config meta = base.hasPath(META_KEY) ? base.getConfig(META_KEY) : com.typesafe.config.ConfigFactory.empty();
        Map<String, String> robots = new LinkedHashMap<>();
        if (meta.hasPath("robots")) {
            ConfigObject obj = meta.getObject("robots");
            for (String mac : obj.keySet()) {
                robots.put(mac.toLowerCase(), (String) obj.get(mac).unwrapped());
            }
        }
        return new Meta(robots,
                meta.hasPath("sim") ? meta.getString("sim") : null,
                meta.hasPath("simExtra") ? meta.getString("simExtra") : null,
                meta.hasPath("debug") && meta.getBoolean("debug"));
    }

    /** Layers override files on top of the base (later files win), then resolves substitutions. */
    public static Config merge(Config base, List<Config> overrides) {
        Config merged = base;
        for (Config override : overrides) {
            merged = override.withFallback(merged);
        }
        return merged.resolve();
    }

    // ---------------------------------------------------------------- helpers

    private static List<String> keysInFileOrder(ConfigObject obj) {
        List<String> keys = new ArrayList<>(obj.keySet());
        keys.sort(Comparator.comparingInt((String k) -> obj.get(k).origin().lineNumber()).thenComparing(k -> k));
        return keys;
    }

    private static String join(String path, String key) {
        return path.isEmpty() ? key : path + "." + key;
    }

    /** "prismarine.conf:12", for pointing students at the exact line. */
    static String where(ConfigValue value) {
        String file = value.origin().filename();
        String name = file == null ? value.origin().description() : new File(file).getName();
        int line = value.origin().lineNumber();
        return line < 0 ? name : name + ":" + line;
    }
}
