package frc.robot.util;

public class NetworkTablesPathUtil {
    public static String join(String base, String... parts) {
        for (String part : parts) {
            base = chopEnd(base) + "/" + chopStart(part);
        }
        return chop(base);
    }

    public static String chop(String s) {
        return chopEnd(chopStart(s));
    }

    public static String chopStart(String s) {
        while (s.startsWith("/")) {
            s = s.substring(1);
        }
        return s;
    }

    public static String chopEnd(String s) {
        while (s.endsWith("/")) {
            s = s.substring(0, s.length() - 1);
        }
        return s;
    }
}
