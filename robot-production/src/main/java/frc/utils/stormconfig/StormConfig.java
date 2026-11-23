package frc.utils.stormconfig;

import com.typesafe.config.Config;
import com.typesafe.config.ConfigFactory;
import com.typesafe.config.ConfigObject;
import com.typesafe.config.ConfigParseOptions;
import com.typesafe.config.ConfigRenderOptions;
import com.typesafe.config.ConfigUtil;
import com.typesafe.config.ConfigValue;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

import frc.utils.stormconfig.ConfigSchema.ConfigSchemaException;
import frc.utils.stormconfig.ConfigSchema.ListShape;
import frc.utils.stormconfig.ConfigSchema.ObjectShape;

import java.io.File;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Function;

/**
 * Loads the robot's configuration: config.conf, then this robot's file on top of it, then (in
 * simulation) the sim file on top of that. The generated {@code Constants} class reads every value
 * through {@link #root()}.
 *
 * <p>Which robot file is used:
 * <ol>
 *   <li>in simulation, {@code stormconfig.sim} plus {@code stormconfig.simExtra};
 *   <li>otherwise, {@code robot = "file.conf"} in ~/.stormrc.conf if that file exists;
 *   <li>otherwise, the {@code stormconfig.robots} entry matching a MAC address of any network interface;
 *   <li>otherwise, config.conf alone (which fails if config.conf has "@type" placeholders).
 * </ol>
 */
public final class StormConfig {
    private static final String BASE_NAME = "config.conf";
    private static final File RC_FILE = new File(System.getProperty("user.home"), ".stormrc.conf");

    private static Node root;

    private StormConfig() {}

    /** A loaded config object together with its shape, so values come back as the right Java types. */
    public static final class Node {
        private final Config config;
        private final ObjectShape shape;

        Node(Config config, ObjectShape shape) {
            this.config = config;
            this.shape = shape;
        }

        /** Reads the value at a dotted path, already converted (int, double, Angle, Pose2d, List...). */
        @SuppressWarnings("unchecked")
        public <T> T get(String path) {
            return (T) ConfigSchema.decode(shape.lookup(ConfigUtil.splitPath(path)), config.getValue(path));
        }

        /** The object at a dotted path, for reading its fields. */
        public Node node(String path) {
            return new Node(config.getConfig(path), (ObjectShape) shape.lookup(ConfigUtil.splitPath(path)));
        }

        /** A list of objects, each turned into a Java value by {@code element} (the generated records use this). */
        public <T> List<T> getList(String path, Function<Node, T> element) {
            ObjectShape elementShape = (ObjectShape) ((ListShape) shape.lookup(ConfigUtil.splitPath(path))).element();
            List<T> result = new ArrayList<>();
            for (ConfigValue value : config.getList(path)) {
                result.add(element.apply(new Node(((ConfigObject) value).toConfig(), elementShape)));
            }
            return Collections.unmodifiableList(result);
        }
    }

    /** The robot's config, loaded on first use. */
    public static synchronized Node root() {
        if (root == null) {
            root = load(Filesystem.getDeployDirectory());
        }
        return root;
    }

    private static Node load(File deployDir) {
        Config base = parse(new File(deployDir, BASE_NAME));
        ConfigSchema.Meta meta = ConfigSchema.meta(base);

        List<String> files = new ArrayList<>();
        if (RobotBase.isSimulation()) {
            addIfSet(files, meta.sim());
            addIfSet(files, meta.simExtra());
        } else {
            addIfSet(files, robotFile(meta));
        }

        List<Config> overrides = new ArrayList<>();
        for (String file : files) {
            System.out.println("Config: loading " + file + " on top of " + BASE_NAME);
            overrides.add(parse(new File(deployDir, file)));
        }

        // The build already checked every robot file listed in config.conf, so errors here mean a
        // file was edited on the robot. Failing loudly beats running with values nobody intended.
        Config merged = ConfigSchema.merge(base, overrides);
        ObjectShape schema = ConfigSchema.fromBase(base.resolve());
        List<String> errors = ConfigSchema.check(schema, merged);
        if (!errors.isEmpty()) {
            throw new ConfigSchemaException("Robot config " + files, errors);
        }
        if (meta.debug()) {
            System.out.println(merged.root().render(ConfigRenderOptions.defaults().setOriginComments(false)));
        }
        return new Node(merged, schema);
    }

    private static String robotFile(ConfigSchema.Meta meta) {
        if (RC_FILE.exists()) {
            Config rc = parse(RC_FILE);
            if (rc.hasPath("robot")) {
                System.out.println("Config: " + RC_FILE + " selects " + rc.getString("robot"));
                return rc.getString("robot");
            }
        }
        for (String mac : macAddresses()) {
            String file = meta.robots().get(mac);
            if (file != null) {
                System.out.println("Config: MAC address " + mac + " selects " + file);
                return file;
            }
        }
        System.out.println("Config: WARNING no robot file matches MAC addresses " + macAddresses()
                + "; add one to stormconfig.robots in " + BASE_NAME);
        return null;
    }

    /** MAC addresses of every interface, as "00:80:2f:35:54:a0". Interface names differ between roboRIO and SystemCore. */
    private static List<String> macAddresses() {
        List<String> result = new ArrayList<>();
        try {
            for (NetworkInterface ni : Collections.list(NetworkInterface.getNetworkInterfaces())) {
                byte[] mac = ni.getHardwareAddress();
                if (mac == null || mac.length != 6) {
                    continue;
                }
                StringBuilder sb = new StringBuilder();
                for (byte b : mac) {
                    sb.append(sb.length() == 0 ? "" : ":").append(String.format("%02x", b));
                }
                result.add(sb.toString());
            }
        } catch (SocketException e) {
            System.out.println("Config: error reading MAC addresses: " + e.getMessage());
        }
        return result;
    }

    private static void addIfSet(List<String> files, String file) {
        if (file != null && !file.isEmpty()) {
            files.add(file);
        }
    }

    private static Config parse(File file) {
        return ConfigFactory.parseFile(file, ConfigParseOptions.defaults().setAllowMissing(false));
    }
}
