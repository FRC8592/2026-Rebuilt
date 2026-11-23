package frc.utils.stormconfig;

import static edu.wpi.first.units.Units.*;

import com.typesafe.config.ConfigObject;
import com.typesafe.config.ConfigValue;
import com.typesafe.config.ConfigValueType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleFunction;
import java.util.function.DoublePredicate;
import java.util.function.Function;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * The value types the config files can hold beyond HOCON's own scalars: WPILib unit measures
 * (written as strings like {@code "-15 deg"}) and compound types (written as objects tagged with
 * {@code _type}).
 *
 * <p>This file is compiled into both the robot code and the build-time generator, so the
 * generator and the robot always agree on what a value means.
 */
public final class ConfigTypes {
    private ConfigTypes() {}

    /** Key that marks an object as a compound type, e.g. {@code { _type = Pose2d, x = 1.0, y = 2.0 }}. */
    public static final String TYPE_KEY = "_type";

    // ---------------------------------------------------------------- measures

    /** A WPILib measure type, e.g. Angle. {@code javaType} is what the generator writes in the field declaration. */
    public record MeasureType(String javaType, List<String> imports) {
        static MeasureType of(String simpleName) {
            return new MeasureType(simpleName, List.of("edu.wpi.first.units.measure." + simpleName));
        }
    }

    private record UnitDef(MeasureType type, DoubleFunction<Measure<?>> of) {}

    private static final MeasureType DISTANCE = MeasureType.of("Distance");
    private static final MeasureType ANGLE = MeasureType.of("Angle");
    private static final MeasureType VOLTAGE = MeasureType.of("Voltage");
    private static final MeasureType CURRENT = MeasureType.of("Current");
    private static final MeasureType TIME = MeasureType.of("Time");
    private static final MeasureType LINEAR_VELOCITY = MeasureType.of("LinearVelocity");
    private static final MeasureType LINEAR_ACCELERATION = MeasureType.of("LinearAcceleration");
    private static final MeasureType ANGULAR_VELOCITY = MeasureType.of("AngularVelocity");
    private static final MeasureType ANGULAR_ACCELERATION = MeasureType.of("AngularAcceleration");
    private static final MeasureType ANGULAR_JERK = new MeasureType("Velocity<AngularAccelerationUnit>",
            List.of("edu.wpi.first.units.measure.Velocity", "edu.wpi.first.units.AngularAccelerationUnit"));

    /** Unit symbols the config files may use. Add entries here to support more units. */
    private static final Map<String, UnitDef> UNITS = new LinkedHashMap<>();

    static {
        UNITS.put("m", new UnitDef(DISTANCE, Meters::of));
        UNITS.put("cm", new UnitDef(DISTANCE, Centimeters::of));
        UNITS.put("mm", new UnitDef(DISTANCE, Millimeters::of));
        UNITS.put("in", new UnitDef(DISTANCE, Inches::of));
        UNITS.put("ft", new UnitDef(DISTANCE, Feet::of));

        UNITS.put("rad", new UnitDef(ANGLE, Radians::of));
        UNITS.put("deg", new UnitDef(ANGLE, Degrees::of));
        UNITS.put("rot", new UnitDef(ANGLE, Rotations::of));

        UNITS.put("V", new UnitDef(VOLTAGE, Volts::of));
        UNITS.put("A", new UnitDef(CURRENT, Amps::of));

        UNITS.put("s", new UnitDef(TIME, Seconds::of));
        UNITS.put("ms", new UnitDef(TIME, Milliseconds::of));

        UNITS.put("m/s", new UnitDef(LINEAR_VELOCITY, MetersPerSecond::of));
        UNITS.put("ft/s", new UnitDef(LINEAR_VELOCITY, FeetPerSecond::of));
        UNITS.put("m/s^2", new UnitDef(LINEAR_ACCELERATION, MetersPerSecondPerSecond::of));
        UNITS.put("ft/s^2", new UnitDef(LINEAR_ACCELERATION, FeetPerSecondPerSecond::of));

        UNITS.put("rad/s", new UnitDef(ANGULAR_VELOCITY, RadiansPerSecond::of));
        UNITS.put("deg/s", new UnitDef(ANGULAR_VELOCITY, DegreesPerSecond::of));
        UNITS.put("rot/s", new UnitDef(ANGULAR_VELOCITY, RotationsPerSecond::of));
        UNITS.put("rpm", new UnitDef(ANGULAR_VELOCITY, RPM::of));
        UNITS.put("rad/s^2", new UnitDef(ANGULAR_ACCELERATION, RadiansPerSecondPerSecond::of));
        UNITS.put("deg/s^2", new UnitDef(ANGULAR_ACCELERATION, DegreesPerSecondPerSecond::of));
        UNITS.put("rot/s^2", new UnitDef(ANGULAR_ACCELERATION, RotationsPerSecondPerSecond::of));
        UNITS.put("rot/s^3", new UnitDef(ANGULAR_JERK, RotationsPerSecondPerSecond.per(Second)::of));
    }

    /** A number followed by a unit-like token, e.g. "-15 deg", "4.5 m/s", "3000 rot/s^3". */
    private static final Pattern MEASURE_PATTERN = Pattern.compile(
            "^\\s*([-+]?(?:\\d+\\.?\\d*|\\.\\d+)(?:[eE][-+]?\\d+)?)\\s*([A-Za-z][A-Za-z0-9/^]*)\\s*$");

    /** The result of reading a measure string: its type and the WPILib value. */
    public record ParsedMeasure(MeasureType type, Measure<?> value) {}

    /**
     * Reads a string as a measure. Returns null if the string isn't shaped like "number unit"
     * (so it is an ordinary string), and throws if it is shaped like one but the unit is unknown,
     * because that is almost always a typo.
     */
    public static ParsedMeasure parseMeasure(String text) {
        Matcher m = MEASURE_PATTERN.matcher(text);
        if (!m.matches()) {
            return null;
        }
        UnitDef unit = UNITS.get(m.group(2));
        if (unit == null) {
            throw new IllegalArgumentException("unknown unit '" + m.group(2) + "' in \"" + text
                    + "\" (known units: " + String.join(", ", UNITS.keySet()) + ")");
        }
        return new ParsedMeasure(unit.type(), unit.of().apply(Double.parseDouble(m.group(1))));
    }

    // ---------------------------------------------------------------- compound types

    /**
     * One field of a compound type. Measure fields are converted to WPILib base units (meters,
     * radians, ...) before the type is built. {@code bareNumberAllowed} lets a field take a plain
     * number in base units; it is off for angles so "yaw = 90" can't silently mean 90 radians.
     */
    public record Field(String name, MeasureType measure, boolean bareNumberAllowed, Double defaultValue,
            DoublePredicate valid, String validDescription) {

        static Field distance(String name) {
            return new Field(name, DISTANCE, true, null, v -> true, null);
        }

        static Field angle(String name) {
            return new Field(name, ANGLE, false, 0.0, v -> true, null);
        }

        static Field measure(String name, MeasureType type) {
            return new Field(name, type, false, null, v -> true, null);
        }

        static Field colorChannel(String name) {
            return new Field(name, null, true, null, v -> v == Math.rint(v) && v >= 0 && v <= 255,
                    "an integer from 0 to 255");
        }
    }

    /** A type built from a config object. {@code build} receives field values in base units. */
    public record CompoundType(String name, String importName, List<Field> fields,
            Function<Map<String, Double>, Object> build) {}

    private static final Map<String, CompoundType> COMPOUNDS = new LinkedHashMap<>();

    private static void compound(Class<?> type, Function<Map<String, Double>, Object> build, Field... fields) {
        COMPOUNDS.put(type.getSimpleName(),
                new CompoundType(type.getSimpleName(), type.getName(), List.of(fields), build));
    }

    static {
        compound(Translation2d.class, f -> new Translation2d(f.get("x"), f.get("y")),
                Field.distance("x"), Field.distance("y"));
        compound(Pose2d.class, f -> new Pose2d(f.get("x"), f.get("y"), new Rotation2d(f.get("rotation"))),
                Field.distance("x"), Field.distance("y"), Field.angle("rotation"));
        compound(Translation3d.class, f -> new Translation3d(f.get("x"), f.get("y"), f.get("z")),
                Field.distance("x"), Field.distance("y"), Field.distance("z"));
        compound(Rotation3d.class, f -> new Rotation3d(f.get("roll"), f.get("pitch"), f.get("yaw")),
                Field.angle("roll"), Field.angle("pitch"), Field.angle("yaw"));
        compound(Transform3d.class, f -> new Transform3d(
                new Translation3d(f.get("x"), f.get("y"), f.get("z")),
                new Rotation3d(f.get("roll"), f.get("pitch"), f.get("yaw"))),
                Field.distance("x"), Field.distance("y"), Field.distance("z"),
                Field.angle("roll"), Field.angle("pitch"), Field.angle("yaw"));
        compound(Color.class, f -> new Color(f.get("r").intValue(), f.get("g").intValue(), f.get("b").intValue()),
                Field.colorChannel("r"), Field.colorChannel("g"), Field.colorChannel("b"));
        compound(TrajectoryConfig.class, f -> new TrajectoryConfig(f.get("maxVelocity"), f.get("maxAcceleration")),
                Field.measure("maxVelocity", LINEAR_VELOCITY), Field.measure("maxAcceleration", LINEAR_ACCELERATION));
    }

    /** Looks up the compound type named by an object's {@code _type}. Throws if it is missing or unknown. */
    public static CompoundType compoundType(ConfigObject obj) {
        ConfigValue tag = obj.get(TYPE_KEY);
        if (tag.valueType() != ConfigValueType.STRING) {
            throw new IllegalArgumentException(TYPE_KEY + " must be a string naming a type");
        }
        CompoundType type = COMPOUNDS.get((String) tag.unwrapped());
        if (type == null) {
            throw new IllegalArgumentException("unknown " + TYPE_KEY + " '" + tag.unwrapped()
                    + "' (known types: " + String.join(", ", COMPOUNDS.keySet()) + ")");
        }
        return type;
    }

    /**
     * Checks every field of a compound object and returns their values in base units. Collects all
     * problems rather than stopping at the first, so a student sees every mistake in one build.
     */
    public static Map<String, Double> compoundFields(CompoundType type, ConfigObject obj, List<String> errors) {
        Map<String, Double> values = new HashMap<>();
        Map<String, Field> byName = new HashMap<>();
        for (Field field : type.fields()) {
            byName.put(field.name(), field);
        }
        for (String key : obj.keySet()) {
            if (!key.equals(TYPE_KEY) && !byName.containsKey(key)) {
                errors.add("unknown field '" + key + "' for " + type.name() + " (fields: "
                        + String.join(", ", byName.keySet()) + ")");
            }
        }
        for (Field field : type.fields()) {
            ConfigValue value = obj.get(field.name());
            if (value == null) {
                if (field.defaultValue() == null) {
                    errors.add(type.name() + " is missing required field '" + field.name() + "'");
                } else {
                    values.put(field.name(), field.defaultValue());
                }
                continue;
            }
            Double v = fieldValue(type, field, value, errors);
            if (v != null) {
                values.put(field.name(), v);
            }
        }
        return values;
    }

    private static Double fieldValue(CompoundType type, Field field, ConfigValue value, List<String> errors) {
        String where = type.name() + "." + field.name();
        double result;
        if (value.valueType() == ConfigValueType.NUMBER) {
            if (!field.bareNumberAllowed()) {
                errors.add(where + " needs a unit, e.g. \"" + value.unwrapped() + " "
                        + exampleUnit(field.measure()) + "\"");
                return null;
            }
            result = ((Number) value.unwrapped()).doubleValue();
        } else if (value.valueType() == ConfigValueType.STRING && field.measure() != null) {
            ParsedMeasure parsed;
            try {
                parsed = parseMeasure((String) value.unwrapped());
            } catch (IllegalArgumentException e) {
                errors.add(where + ": " + e.getMessage());
                return null;
            }
            if (parsed == null || !parsed.type().equals(field.measure())) {
                errors.add(where + " must be a " + field.measure().javaType() + " like \""
                        + "1 " + exampleUnit(field.measure()) + "\", not \"" + value.unwrapped() + "\"");
                return null;
            }
            result = parsed.value().baseUnitMagnitude();
        } else {
            errors.add(where + " must be " + (field.measure() == null ? "a number" : "a "
                    + field.measure().javaType()) + ", not " + value.valueType().name().toLowerCase());
            return null;
        }
        if (!field.valid().test(result)) {
            errors.add(where + " must be " + field.validDescription() + ", not " + value.unwrapped());
            return null;
        }
        return result;
    }

    private static String exampleUnit(MeasureType type) {
        for (Map.Entry<String, UnitDef> e : UNITS.entrySet()) {
            if (e.getValue().type().equals(type)) {
                return e.getKey();
            }
        }
        return "";
    }

    // ---------------------------------------------------------------- number literals

    private static final java.lang.reflect.Field ORIGINAL_TEXT;

    static {
        try {
            ORIGINAL_TEXT = Class.forName("com.typesafe.config.impl.ConfigNumber").getDeclaredField("originalText");
            ORIGINAL_TEXT.setAccessible(true);
        } catch (ReflectiveOperationException | RuntimeException e) {
            throw new IllegalStateException("This version of com.typesafe:config doesn't keep number text the way"
                    + " ConfigTypes.isDecimalLiteral expects; see ConfigTypesTest", e);
        }
    }

    /**
     * Whether a number was written with a decimal point or exponent ("180.0", "1e3") rather than as
     * a whole number ("180"). Typesafe Config parses 180.0 into the same Integer as 180, and only
     * keeps the text in a non-public field, so this reads that field. ConfigTypesTest pins the
     * behavior so a library upgrade that changes it fails the tests rather than silently retyping values.
     */
    public static boolean isDecimalLiteral(ConfigValue number) {
        if (number.unwrapped() instanceof Double) {
            return true;
        }
        try {
            String text = (String) ORIGINAL_TEXT.get(number);
            return text != null && (text.contains(".") || text.contains("e") || text.contains("E"));
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
    }

    /** The number as written in the file, for error messages. */
    public static String numberText(ConfigValue number) {
        try {
            String text = (String) ORIGINAL_TEXT.get(number);
            return text != null ? text : number.unwrapped().toString();
        } catch (IllegalAccessException e) {
            return number.unwrapped().toString();
        }
    }

    /** The compound type with this name (e.g. "Pose2d"), or null. */
    public static CompoundType compoundType(String name) {
        return COMPOUNDS.get(name);
    }

    /** The measure type with this Java name (e.g. "Angle"), or null. */
    public static MeasureType measureType(String javaType) {
        for (UnitDef unit : UNITS.values()) {
            if (unit.type().javaType().equals(javaType)) {
                return unit.type();
            }
        }
        return null;
    }

    /** Lists the compound types, for error messages and documentation. */
    public static List<String> compoundTypeNames() {
        return new ArrayList<>(COMPOUNDS.keySet());
    }
}
