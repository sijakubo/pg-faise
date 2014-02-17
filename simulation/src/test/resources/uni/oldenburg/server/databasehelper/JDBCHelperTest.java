package uni.oldenburg.server.databasehelper;

import junit.framework.Assert;
import org.junit.Test;

import java.sql.ResultSet;

public class JDBCHelperTest {

    @Test
    public void testExecuteDbStatement() throws Exception {
        ResultSet testResultSet = JDBCHelper.executeSql("select 1");
        Assert.assertTrue(testResultSet.next());
        int testInt = testResultSet.getInt(1);
        Assert.assertEquals(1, testInt);
    }
}
