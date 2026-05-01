package frc.robot.util;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class NetworkTablesPathUtilTest {
    @Test
    public void testChopStart() {
        assertEquals("table", NetworkTablesPathUtil.chopStart("/table"));
        assertEquals("table/sub", NetworkTablesPathUtil.chopStart("table/sub"));

        assertEquals("table", NetworkTablesPathUtil.chopStart("///table"));

        assertEquals("table", NetworkTablesPathUtil.chopStart("table"));

        assertEquals("", NetworkTablesPathUtil.chopStart("///"));
        assertEquals("", NetworkTablesPathUtil.chopStart(""));
    }

    @Test
    public void testChopEnd() {
        assertEquals("table", NetworkTablesPathUtil.chopEnd("table/"));
        assertEquals("table/sub", NetworkTablesPathUtil.chopEnd("table/sub/"));

        assertEquals("table", NetworkTablesPathUtil.chopEnd("table///"));

        assertEquals("table", NetworkTablesPathUtil.chopEnd("table"));

        assertEquals("", NetworkTablesPathUtil.chopEnd("///"));
        assertEquals("", NetworkTablesPathUtil.chopEnd(""));
    }

    @Test
    public void testChop() {
        assertEquals("table", NetworkTablesPathUtil.chop("/table/"));

        assertEquals("table/sub", NetworkTablesPathUtil.chop("///table/sub///"));
        assertEquals("", NetworkTablesPathUtil.chop("///"));
        assertEquals("table", NetworkTablesPathUtil.chop("table"));
    }

    @Test
    public void testJoin() {
        assertEquals("table/subtable", NetworkTablesPathUtil.join("table", "subtable"));

        assertEquals("table/subtable", NetworkTablesPathUtil.join("table/", "/subtable"));
        assertEquals("table/subtable", NetworkTablesPathUtil.join("table///", "///subtable"));

        assertEquals("table/sub1/sub2", NetworkTablesPathUtil.join("table", "sub1", "sub2"));

        assertEquals("table/sub1/sub2", NetworkTablesPathUtil.join("/table/", "/sub1/", "/sub2/"));
    }
}
