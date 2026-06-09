package frc.robot.util;

/**
 * A class that provides tools to ensure a NetworkTables key is formatted correctly
 */
public class NetworkTablesPathUtil {
    /**
     * Join two or more {@link String} objects to form a key
     *
     * This automatically adds "/" between the base and each part
     * @param base The highest-level directory
     * @param parts The subdirectories
     * @return
     */
    public static String join(String base, String... parts) {
        for (String part : parts) {
            base = chopEnd(base) + "/" + chopStart(part);
        }
        return chop(base);
    }

    /**
     * Remove preceding and succeeding "/" characters
     * @param s
     * @return
     */
    public static String chop(String s) {
        return chopEnd(chopStart(s));
    }

    /**
     * Remove preceding "/" characters
     * @param s
     * @return
     */
    public static String chopStart(String s) {
        while (s.startsWith("/")) {
            s = s.substring(1);
        }
        return s;
    }

    /**
     * Remove succeeding "/" characters
     * @param s
     * @return
     */
    public static String chopEnd(String s) {
        while (s.endsWith("/")) {
            s = s.substring(0, s.length() - 1);
        }
        return s;
    }
}
