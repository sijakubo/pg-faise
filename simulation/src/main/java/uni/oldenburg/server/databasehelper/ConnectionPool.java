package uni.oldenburg.server.databasehelper;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;

import org.apache.log4j.Logger;

public final class ConnectionPool {
    private static Connection connection;

    private ConnectionPool() {
    }

    public static Connection getConnection() {
        try {
            if (connection == null || connection.isClosed()) {
                Class.forName("org.postgresql.Driver");
                connection = DriverManager.getConnection("jdbc:postgresql://134.106.13.165:5432/faisedb", "faiseuser", "sonne15");
            }
        } catch (ClassNotFoundException e) {
            Logger logger = Logger.getLogger(ConnectionPool.class);
            logger.error("Unable to find Postgres-Database Driver", e);
        } catch (SQLException e) {
            Logger logger = Logger.getLogger(ConnectionPool.class);
            logger.error("Unable to estable connection to Postgres-Database", e);
        }

        return connection;
    }
}
