package uni.oldenburg.server.dao;

import uni.oldenburg.server.databasehelper.ConnectionPool;
import uni.oldenburg.shared.model.SimulationUser;

import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;

public class SimulationUserDao {

   /**
    * @author sijakubo
    */
    public SimulationUser findSimulationUserById(Long userId) throws SQLException {
        PreparedStatement preparedStatement = ConnectionPool.getConnection()
                .prepareStatement("SELECT id, name, password, email FROM " + SimulationUser.TABLE_NAME + " WHERE id = ?");

        preparedStatement.setLong(1, userId);
        ResultSet resultSet = preparedStatement.executeQuery();

        if (!resultSet.next()) {
            throw new IllegalStateException("Unable to find User for UserId: " + userId);
        }

        SimulationUser simulationUser = new SimulationUser(
                resultSet.getLong("id"),
                resultSet.getString("name"),
                resultSet.getString("email"),
                resultSet.getString("password"));

        resultSet.close();
        preparedStatement.close();

        return simulationUser;
    }

   /**
    * @author sijakubo
    */
    public void persistSimulationUser(SimulationUser newUser) throws SQLException {


        PreparedStatement prepStatement = ConnectionPool.getConnection()
                .prepareStatement("INSERT INTO " + SimulationUser.TABLE_NAME + " (email, name, password) VALUES (?, ?, ?)");

        prepStatement.setString(1, newUser.getEmail());
        prepStatement.setString(2, newUser.getName());
        prepStatement.setString(3, newUser.getPassword());
        prepStatement.executeUpdate();
    }

    public SimulationUser findUserForUsernameAndPassword(String email, String password) throws SQLException {
        PreparedStatement prepStatement = ConnectionPool.getConnection()
                .prepareStatement("SELECT id, email, name, password"
                        + " FROM " + SimulationUser.TABLE_NAME
                        + " WHERE email = ? and password = ?");

        prepStatement.setString(1, email);
        prepStatement.setString(2, password);
        ResultSet resultSet = prepStatement.executeQuery();

        SimulationUser user = new SimulationUser();
        if (resultSet.next()) {
            //User found for email and Password
            user.setId(resultSet.getLong("id"));
            user.setEmail(resultSet.getString("email"));
            user.setName(resultSet.getString("name"));
            user.setPassword(resultSet.getString("password"));
        } else {
            System.out.println("No user Found!");
            user = null;
        }

        return user;
    }
}
