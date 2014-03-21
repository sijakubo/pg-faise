package uni.oldenburg.server.dao;

import uni.oldenburg.server.databasehelper.ConnectionPool;
import uni.oldenburg.shared.model.*;

import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.List;

public class SzenarioDao {

	// Returns the names of the Szenarios so that they can be showed in the
	// Popup for Scenario selection
	public ArrayList<String> getSzenarioTitles() throws SQLException {

		ArrayList<String> scenarioTitles = new ArrayList<String>();

		String strSQL = "SELECT title FROM " + Szenario.TABLE_NAME;

		PreparedStatement preparedStatement = ConnectionPool.getConnection()
				.prepareStatement(strSQL);

		ResultSet resultSet = preparedStatement.executeQuery();

		while (resultSet.next()) {

			String title = resultSet.getString("title");
			
			scenarioTitles.add(title);

		}

		return scenarioTitles;

	}
	
	//Persists a Szenario into the Database
	public void persistSzenario(Szenario szenario) throws SQLException {

		//Todo unterscheiden zw. Insert und Update

        PreparedStatement prepStatement = ConnectionPool.getConnection()
                .prepareStatement("INSERT INTO " + Szenario.TABLE_NAME + " (id, title, time_created, created_by_user) VALUES (?, ?, ?, ?)");

        prepStatement.setInt(1, szenario.getID());
        prepStatement.setString(2, szenario.getTitle());
        prepStatement.setString(3, szenario.getTimeCreated());
        prepStatement.setString(4, szenario.getCreatedByUser());
        
        prepStatement.executeUpdate();
        
        
        
        
        prepStatement = ConnectionPool.getConnection()
                .prepareStatement("INSERT INTO " + Conveyor.TABLE_NAME + " (szenario_id, type, pos_x, pos_y) VALUES (?, ?, ?, ?)");
        
        
        //Persist the Conveyors
        List<Conveyor> lstConveyor = szenario.getConveyorList();
        
        for (Conveyor myConveyor : lstConveyor) {
        	
        	prepStatement.setInt(1, myConveyor.getID());
            prepStatement.setString(2, myConveyor.getType());
            prepStatement.setInt(3, myConveyor.getX());
            prepStatement.setInt(4, myConveyor.getY());
            
            prepStatement.executeUpdate();
        	
			
		}
        
        
    }
	
	
	
	
	
	
	
	

	public Szenario loadSzenario(String name) throws SQLException {
		String strSQL = "SELECT szenario.id AS id, time_created, simulationuser.name AS user_name "
				+ "FROM "
				+ Szenario.TABLE_NAME
				+ ", "
				+ SimulationUser.TABLE_NAME
				+ " "
				+ "WHERE szenario.user_id = simulationuser.id AND szenario.title = ?";

		PreparedStatement preparedStatement = ConnectionPool.getConnection()
				.prepareStatement(strSQL);

		preparedStatement.setString(1, name);
		ResultSet resultSet = preparedStatement.executeQuery();

		if (!resultSet.next()) {
			throw new IllegalStateException("Szenario could not be found: "
					+ name);
		}

		Szenario newSzenario = new Szenario(resultSet.getInt("id"), name,
				resultSet.getString("time_created"),
				resultSet.getString("user_name"));

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

		String strSQL = "SELECT type, pos_x, pos_y " + "FROM "
				+ Conveyor.TABLE_NAME + " " + "WHERE szenario_id = ?";

		PreparedStatement preparedStatement = ConnectionPool.getConnection()
				.prepareStatement(strSQL);

		preparedStatement.setInt(1, szenario_id);
		ResultSet resultSet = preparedStatement.executeQuery();

		while (resultSet.next()) {
			Conveyor newConveyor = null;

			String type = resultSet.getString("type");
			int posX = resultSet.getInt("pos_x");
			int posY = resultSet.getInt("pos_y");

			if (type.compareTo(ConveyorRamp.TYPE) == 0) {
				newConveyor = new ConveyorRamp(posX, posY);
			} else if (type.compareTo(ConveyorVehicle.TYPE) == 0) {
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
