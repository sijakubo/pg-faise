package uni.oldenburg.server.dao;

import uni.oldenburg.server.databasehelper.ConnectionPool;
import uni.oldenburg.shared.model.*;

import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.List;

public class SzenarioDao {
    public Szenario loadSzenario(String name) throws SQLException {
        PreparedStatement preparedStatement = ConnectionPool.getConnection()
                .prepareStatement("SELECT szenario.id AS id, time_created, simulationuser.name AS user_name " +
                        "FROM " + Szenario.TABLE_NAME + ", " + SimulationUser.TABLE_NAME + " " +
                        "WHERE szenario.user_id = simulationuser.id AND szenario.title = ?");

        preparedStatement.setString(1, name);
        ResultSet resultSet = preparedStatement.executeQuery();

        if (!resultSet.next()) {
            throw new IllegalStateException("Szenario could not be found: " + name);
        }

        Szenario newSzenario =
                new Szenario(resultSet.getInt("id"), name, resultSet.getString("time_created"), resultSet.getString("user_name"));

        List<Conveyor> lstConveyor = loadConveyor(newSzenario.getID());

        for (Conveyor newConveyor : lstConveyor) {
            newSzenario.addConveyor(newConveyor);
        }

        resultSet.close();
        preparedStatement.close();

        return newSzenario;
    }

    private List<Conveyor> loadConveyor(int szenario_id) throws SQLException {
        List<Conveyor> lstConveyor = new ArrayList<Conveyor>();

        String strSQL = "SELECT type, pos_x, pos_y " +
                "FROM " + Conveyor.TABLE_NAME + " " +
                "WHERE szenario_id = ?";

        PreparedStatement preparedStatement = ConnectionPool.getConnection().prepareStatement(strSQL);

        preparedStatement.setInt(1, szenario_id);
        ResultSet resultSet = preparedStatement.executeQuery();

        while (resultSet.next()) {
            Conveyor newConveyor = null;
            String type = resultSet.getString("type");
            int posX = resultSet.getInt("pos_x");
            int posY = resultSet.getInt("pos_y");

            newConveyor = new ConveyorVehicle(0, 0);

            if (type == ConveyorRamp.TYPE) {
                newConveyor = new ConveyorRamp(posX, posY);
            } else if (type == ConveyorVehicle.TYPE) {
                newConveyor = new ConveyorVehicle(posX, posY);
            }

            if (newConveyor != null)
                lstConveyor.add(newConveyor);
        }

        resultSet.close();
        preparedStatement.close();

        return lstConveyor;
    }
}
