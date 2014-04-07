package uni.oldenburg.server.databasehelper;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;

/**
 * @author sijakubo
 */
public final class ConnectionPool {
   private static Connection connection;

   private ConnectionPool() {
   }

   /**
    * @author sijakubo
    */
   public static Connection getConnection() {
      try {
         if (connection == null || connection.isClosed()) {
            Logger logger = Logger.getLogger(ConnectionPool.class);
            logger.log(Level.INFO, "Connection established to Database");

            Class.forName("org.postgresql.Driver");
            connection = DriverManager.getConnection("jdbc:postgresql://134.106.13.165:5432/faisedb", "faiseuser", "sonne15");
         }
      } catch (ClassNotFoundException e) {
         Logger logger = Logger.getLogger(ConnectionPool.class);
         logger.log(Level.ERROR, "Unable to find Postgres-Database Driver", e);
      } catch (SQLException e) {
         Logger logger = Logger.getLogger(ConnectionPool.class);
         logger.log(Level.ERROR, "Unable to estable connection to Postgres-Database", e);
      }

      return connection;
   }
}
