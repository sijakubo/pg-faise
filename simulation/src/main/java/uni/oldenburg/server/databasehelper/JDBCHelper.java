package uni.oldenburg.server.databasehelper;

import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;

import org.apache.log4j.Logger;

public final class JDBCHelper {
    private JDBCHelper() {
    }

    public static ResultSet executeSql(String sqlStatement) {
        try {
            Statement statement = ConnectionPool.getConnection().createStatement();
            statement.execute(sqlStatement);

            return statement.getResultSet();
        } catch (SQLException e) {
            Logger logger = Logger.getLogger(JDBCHelper.class);
            logger.error("Unable to execute SQL-Statement", e);
        }

        return null;
    }
}
