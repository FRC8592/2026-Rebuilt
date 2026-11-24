package frc.utils.stormconfig;

import com.typesafe.config.Config;
import com.typesafe.config.ConfigException;
import com.typesafe.config.ConfigList;
import com.typesafe.config.ConfigObject;
import com.typesafe.config.ConfigUtil;
import com.typesafe.config.ConfigValue;
import com.typesafe.config.ConfigValueType;

import frc.utils.stormconfig.ConfigSchema.ConfigSchemaException;
import frc.utils.stormconfig.ConfigSchema.MeasureShape;
import frc.utils.stormconfig.ConfigSchema.Meta;
import frc.utils.stormconfig.ConfigSchema.ObjectShape;
import frc.utils.stormconfig.ConfigSchema.Scalar;
import frc.utils.stormconfig.ConfigSchema.ScalarKind;
import frc.utils.stormconfig.ConfigSchema.Shape;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;

/**
 * The config files that make up the project: config.conf, the robot files listed in its
 * {@code stormconfig} block, and the sim file. The rules:
 * <ul>
 *   <li>the schema is the union of config.conf and every robot file, so a value only some robots
 *       need can live in just those robots' files;
 *   <li>a key has one type everywhere it appears;
 *   <li>keys that differ only by case or underscores (SNAP_TO_kP / SNAP_TO_KP) are rejected as typos;
 *   <li>the sim file only overrides: it can change the sim robot's values but not add keys.
 * </ul>
 * A robot may leave keys out; code that uses a section with a missing key fails with a clear
 * message when that section is first used (see {@link StormConfig.Node#require}).
 *
 * <p>Compiled into both the robot code and the build-time generator.
 */
public final class ConfigProject {
    public static final String BASE_NAME = "config.conf";

    private final Config base;
    private final Meta meta;
    private final Map<String, Config> files = new LinkedHashMap<>();
    private final Map<List<String>, Config> merged = new HashMap<>();

    /** {@code parse} reads a file in the deploy directory by name, unresolved. */
    public ConfigProject(Function<String, Config> parse) {
        base = parse.apply(BASE_NAME);
        meta = ConfigSchema.meta(base);
        for (String file : robotFiles()) {
            files.put(file, parse.apply(file));
        }
        if (meta.simExtra() != null) {
            files.put(meta.simExtra(), parse.apply(meta.simExtra()));
        }
    }

    public Meta meta() {
        return meta;
    }

    /** Robot files in the union: every listed robot, plus the sim robot. */
    private Set<String> robotFiles() {
        Set<String> robots = new LinkedHashSet<>(meta.robots().values());
        if (meta.sim() != null) {
            robots.add(meta.sim());
        }
        return robots;
    }

    /** The file lists the robot can run with: config.conf alone, each robot, and the sim robot plus the sim file. */
    public List<List<String>> runtimeCombinations() {
        List<List<String>> combos = new ArrayList<>();
        combos.add(List.of());
        for (String robot : meta.robots().values()) {
            combos.add(List.of(robot));
        }
        if (meta.sim() != null) {
            combos.add(meta.simExtra() == null ? List.of(meta.sim()) : List.of(meta.sim(), meta.simExtra()));
        }
        return combos;
    }

    /** "config.conf + prismarine.conf", for messages. */
    public static String describe(List<String> fileNames) {
        return fileNames.isEmpty() ? BASE_NAME : BASE_NAME + " + " + String.join(" + ", fileNames);
    }

    /** config.conf with the named files layered on top, resolved. */
    public Config merged(List<String> fileNames) {
        return merged.computeIfAbsent(List.copyOf(fileNames), names -> {
            List<Config> overrides = new ArrayList<>();
            for (String name : names) {
                Config file = files.get(name);
                if (file == null) {
                    throw new IllegalArgumentException(name + " is not listed in the stormconfig block of " + BASE_NAME);
                }
                overrides.add(file);
            }
            return ConfigSchema.merge(base, overrides);
        });
    }

    // ---------------------------------------------------------------- the schema

    /** The union schema. Throws with every problem if the files disagree. */
    public ObjectShape schema() {
        List<String> errors = new ArrayList<>();
        Map<String, Shape> union = new LinkedHashMap<>();
        Map<String, String> firstSeen = new HashMap<>();

        List<List<String>> trees = new ArrayList<>();
        trees.add(List.of());
        for (String robot : robotFiles()) {
            trees.add(List.of(robot));
        }
        for (List<String> tree : trees) {
            Config config = merged(tree);
            List<String> shapeErrors = new ArrayList<>();
            ObjectShape shape = ConfigSchema.shapeOfTree(config, shapeErrors);
            shapeErrors.forEach(e -> errors.add("[" + describe(tree) + "] " + e));
            addToUnion(union, shape, config, "", firstSeen, errors);
        }
        ObjectShape schema = new ObjectShape(union);
        checkNearMisses(schema, "", firstSeen, errors);
        if (!errors.isEmpty()) {
            throw new ConfigSchemaException("The config files", dedupe(errors));
        }
        return schema;
    }

    private static void addToUnion(Map<String, Shape> union, ObjectShape shape, Config tree, String path,
            Map<String, String> firstSeen, List<String> errors) {
        for (Map.Entry<String, Shape> entry : shape.fields().entrySet()) {
            String key = entry.getKey();
            String fullPath = path.isEmpty() ? key : path + "." + key;
            Shape incoming = entry.getValue();
            Shape existing = union.get(key);
            if (existing == null) {
                if (incoming instanceof ObjectShape obj) {
                    Map<String, Shape> fields = new LinkedHashMap<>();
                    union.put(key, new ObjectShape(fields));
                    addToUnion(fields, obj, tree, fullPath, firstSeen, errors);
                } else {
                    union.put(key, incoming);
                }
                firstSeen.putIfAbsent(fullPath, ConfigSchema.where(tree.getValue(fullPath)));
            } else if (existing instanceof ObjectShape e && incoming instanceof ObjectShape i) {
                addToUnion(e.fields(), i, tree, fullPath, firstSeen, errors);
            } else if (!existing.equals(incoming)) {
                errors.add(ConfigSchema.typeMismatch(fullPath, existing, tree.getValue(fullPath))
                        + " (it is " + existing.describe() + " at " + firstSeen.get(fullPath) + "; a key has one type in every file)");
            }
        }
    }

    /** SNAP_TO_kP and SNAP_TO_KP side by side is almost certainly a typo. */
    private static void checkNearMisses(ObjectShape shape, String path, Map<String, String> firstSeen, List<String> errors) {
        Map<String, String> normalized = new HashMap<>();
        for (Map.Entry<String, Shape> entry : shape.fields().entrySet()) {
            String key = entry.getKey();
            String fullPath = path.isEmpty() ? key : path + "." + key;
            String norm = key.replace("_", "").toLowerCase();
            String other = normalized.putIfAbsent(norm, key);
            if (other != null) {
                String otherPath = path.isEmpty() ? other : path + "." + other;
                errors.add(firstSeen.get(fullPath) + " " + fullPath + ": looks like a typo of " + otherPath + " ("
                        + firstSeen.get(otherPath) + "); keys can't differ only by case or underscores");
            }
            if (entry.getValue() instanceof ObjectShape obj) {
                checkNearMisses(obj, fullPath, firstSeen, errors);
            }
        }
    }

    // ---------------------------------------------------------------- checking what a robot loads

    /** Checks one runtime combination against the schema, including the sim file's override-only rule. */
    public List<String> check(ObjectShape schema, List<String> fileNames) {
        List<String> errors = new ArrayList<>();
        int last = fileNames.size() - 1;
        if (last >= 0 && fileNames.get(last).equals(meta.simExtra())) {
            Config under = merged(fileNames.subList(0, last));
            ConfigSchema.checkOverrideOnly(files.get(meta.simExtra()).root(), under.root(), "", errors);
            if (!errors.isEmpty()) {
                // The same keys would also show up below as unknown; the sim-file message is the useful one
                return errors;
            }
        }
        errors.addAll(ConfigSchema.check(schema, merged(fileNames)));
        return errors;
    }

    /** Checks every runtime combination; returns every problem labeled with its file list. */
    public List<String> checkAll(ObjectShape schema) {
        List<String> errors = new ArrayList<>();
        for (List<String> combo : runtimeCombinations()) {
            for (String error : check(schema, combo)) {
                errors.add("[" + describe(combo) + "] " + error);
            }
        }
        return dedupe(errors);
    }

    // ---------------------------------------------------------------- dashboard tunables

    private record Declared(String section, String name, ConfigValue at) {}

    /**
     * Which values are tunable from the dashboard: section path to key names, in schema order. A
     * section lists its own, e.g. {@code TURRET { _tunables = ["TURRET_P"] }}. Lists in config.conf and
     * the robot files add up, like keys do, so a robot file can make its own values tunable; the
     * sim file can only repeat values that are already tunable. Throws with every problem found.
     */
    public Map<String, List<String>> tunables(ObjectShape schema) {
        List<String> errors = new ArrayList<>();
        List<Declared> declared = new ArrayList<>();
        collectTunables(base.root(), "", declared, errors);
        for (String file : robotFiles()) {
            collectTunables(files.get(file).root(), "", declared, errors);
        }
        Map<String, Set<String>> bySection = new HashMap<>();
        for (Declared d : declared) {
            Shape shape = find(schema, d.section(), d.name());
            String at = ConfigSchema.where(d.at()) + " " + d.section() + "." + ConfigSchema.TUNABLES_KEY + ": ";
            if (shape == null) {
                errors.add(at + d.section() + " has no value " + d.name());
            } else if (!(shape instanceof MeasureShape || shape instanceof Scalar s && s.kind() != ScalarKind.LONG)) {
                errors.add(at + d.name() + " is " + shape.describe()
                        + "; tunables can be boolean, int, double, String or a unit type like \"40 V\"");
            } else {
                bySection.computeIfAbsent(d.section(), k -> new HashSet<>()).add(d.name());
            }
        }
        if (meta.simExtra() != null) {
            List<Declared> sim = new ArrayList<>();
            collectTunables(files.get(meta.simExtra()).root(), "", sim, errors);
            for (Declared d : sim) {
                if (!bySection.getOrDefault(d.section(), Set.of()).contains(d.name())) {
                    errors.add(ConfigSchema.where(d.at()) + " " + d.section() + "." + ConfigSchema.TUNABLES_KEY + ": "
                            + d.name() + " isn't tunable in config.conf or a robot file; sim files can't add tunables");
                }
            }
        }
        if (!errors.isEmpty()) {
            throw new ConfigSchemaException("The config files", dedupe(errors));
        }
        Map<String, List<String>> result = new LinkedHashMap<>();
        orderTunables(schema, "", bySection, result);
        return result;
    }

    private static void collectTunables(ConfigObject obj, String path, List<Declared> out, List<String> errors) {
        for (String key : obj.keySet()) {
            ConfigValue value = obj.get(key);
            if (path.isEmpty() && key.equals(ConfigSchema.META_KEY)) {
                continue;
            }
            if (key.equals(ConfigSchema.TUNABLES_KEY)) {
                String at = ConfigSchema.where(value) + " " + (path.isEmpty() ? "" : path + ".") + key + ": ";
                if (path.isEmpty()) {
                    errors.add(at + "goes inside a section, e.g. TURRET { " + key + " = [\"TURRET_P\"] }");
                } else if (!(value instanceof ConfigList list)) {
                    errors.add(at + "should be a list of this section's keys, e.g. [\"TURRET_P\", \"TURRET_D\"]");
                } else {
                    for (ConfigValue element : list) {
                        if (isString(element)) {
                            out.add(new Declared(path, (String) element.unwrapped(), element));
                        } else {
                            errors.add(at + "should be a list of this section's keys, e.g. [\"TURRET_P\", \"TURRET_D\"]");
                        }
                    }
                }
            } else if (value instanceof ConfigObject child && !child.containsKey(ConfigTypes.TYPE_KEY)) {
                collectTunables(child, path.isEmpty() ? key : path + "." + key, out, errors);
            }
        }
    }

    private static boolean isString(ConfigValue value) {
        try {
            return value.valueType() == ConfigValueType.STRING;
        } catch (ConfigException.NotResolved e) {
            return false; // a ${substitution}; tunable names must be written out
        }
    }

    /** The shape of {@code section.name}, or null if there is no such value. */
    private static Shape find(ObjectShape schema, String section, String name) {
        Shape shape = schema;
        for (String key : ConfigUtil.splitPath(section)) {
            if (!(shape instanceof ObjectShape obj)) {
                return null;
            }
            shape = obj.fields().get(key);
        }
        return shape instanceof ObjectShape obj ? obj.fields().get(name) : null;
    }

    private static void orderTunables(ObjectShape shape, String path, Map<String, Set<String>> bySection,
            Map<String, List<String>> result) {
        Set<String> names = bySection.get(path);
        if (names != null) {
            result.put(path, shape.fields().keySet().stream().filter(names::contains).toList());
        }
        shape.fields().forEach((key, field) -> {
            if (field instanceof ObjectShape obj) {
                orderTunables(obj, path.isEmpty() ? key : path + "." + key, bySection, result);
            }
        });
    }

    // ---------------------------------------------------------------- documentation

    /** Where a key is first defined: config.conf if there, otherwise the first robot file that has it. */
    public record Definition(String file, ConfigValue source, ConfigValue resolved) {}

    public Definition definition(String path) {
        List<String> order = new ArrayList<>();
        order.add(null);
        order.addAll(robotFiles());
        for (String file : order) {
            ConfigValue source = find(file == null ? base.root() : files.get(file).root(), path);
            if (source != null) {
                Config tree = merged(file == null ? List.of() : List.of(file));
                return new Definition(file == null ? BASE_NAME : file, source, tree.getValue(path));
            }
        }
        throw new IllegalArgumentException("no config key " + path);
    }

    private static ConfigValue find(ConfigObject root, String path) {
        ConfigValue value = root;
        for (String key : ConfigUtil.splitPath(path)) {
            if (!(value instanceof ConfigObject obj) || !obj.containsKey(key)) {
                return null;
            }
            value = obj.get(key);
        }
        return value;
    }

    private static List<String> dedupe(List<String> errors) {
        return new ArrayList<>(new LinkedHashSet<>(errors));
    }
}
