package uni.oldenburg.server.dao;

import uni.oldenburg.server.databasehelper.ConnectionPool;
import uni.oldenburg.shared.model.*;

import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.List;

public class SzenarioDao {

	/**
	 * Checks wether a szenario with the given title exists or not
	 * 
	 * @author Raschid
	 */
	public boolean checkIfTitleExists(String title) throws SQLException {
		String strSQL = "SELECT * FROM " + Szenario.TABLE_NAME
				+ " WHERE szenario.title=?";

		PreparedStatement preparedStatement = ConnectionPool.getConnection()
				.prepareStatement(strSQL);

		preparedStatement.setString(1, title);

		ResultSet resultSet = preparedStatement.executeQuery();

		if (resultSet.next()) {
			return true;
		} else
			return false;

	}

	/**
	 * Returns the names of all Szenarios so that they can be showed in the
	 * Popup for Scenario selection
	 * 
	 * @author Raschid
	 */
	public ArrayList<String> getSzenarioTitles() throws SQLException {

		ArrayList<String> scenarioTitles = new ArrayList<String>();

		String strSQL = "SELECT title FROM " + Szenario.TABLE_NAME + " ORDER BY title";

		PreparedStatement preparedStatement = ConnectionPool.getConnection()
				.prepareStatement(strSQL);

		ResultSet resultSet = preparedStatement.executeQuery();

		while (resultSet.next()) {

			String title = resultSet.getString("title");

			scenarioTitles.add(title);

		}

		return scenarioTitles;

	}

	/**
	 * Persists a Szenario into the Database by executing an Insert
	 * 
	 * @author Raschid Matthias
	 */
	public void persistSzenario(Szenario szenario) throws SQLException {
		// delete conveyors of szenario
        PreparedStatement prepStatement = ConnectionPool
                .getConnection()
                .prepareStatement("DELETE FROM szenario_conveyor WHERE szenario_id = (SELECT id from szenario WHERE title = ?)");

        prepStatement.setString(1, szenario.getTitle());
        prepStatement.executeUpdate();

        // delete current szenario
        prepStatement = ConnectionPool
                .getConnection()
                .prepareStatement("DELETE FROM szenario WHERE title = ?");

        prepStatement.setString(1, szenario.getTitle());
        prepStatement.executeUpdate();

		prepStatement = ConnectionPool
				.getConnection()
				.prepareStatement(
						"INSERT INTO szenario (" +
							"id, " +
							"title, " +
							"time_created, " +
							"user_id) " +
						"VALUES (" +
							"(SELECT CASE WHEN MAX(ID) IS NULL THEN 0 ELSE MAX(ID)+1 END FROM szenario), " +
							"?, " +
							"now(), " +
							"4)");
		
			// use this as soon as it's possible to get user information out of session				
		//"(SELECT ID FROM simulationuser WHERE name = ?)");

		prepStatement.setString(1, szenario.getTitle());

		prepStatement.executeUpdate();

		// Inserting the Szenariodata
		prepStatement = ConnectionPool
				.getConnection()
				.prepareStatement(
						"INSERT INTO " + Conveyor.TABLE_NAME +
							" (szenario_id, type, pos_x, pos_y, direction) " +
						"VALUES" + 
							"((SELECT id FROM szenario WHERE title = ?), ?, ?, ?, ?)");

		// Persisting the Conveyors
		List<Conveyor> lstConveyor = szenario.getConveyorList();

		for (Conveyor myConveyor : lstConveyor) {
			prepStatement.setString(1, szenario.getTitle());
			prepStatement.setString(2, myConveyor.getType());
			prepStatement.setInt(3, myConveyor.getX());
			prepStatement.setInt(4, myConveyor.getY());
			prepStatement.setInt(5, myConveyor.getDirection());

			prepStatement.executeUpdate();
		}
	}

	/**
	 * loads the Szenario by a name and intializes the Szenario and Conveyor
	 * Objects with
	 * 
	 * @author Raschid Matthias
	 */
	public Szenario loadSzenario(String name) throws SQLException {
		String strSQL = "SELECT szenario.id AS id, time_created, simulationuser.name AS user_name "
				+ "FROM "
				+ Szenario.TABLE_NAME
				+ ", "
				+ SimulationUser.TABLE_NAME
				+ " "
				+ "WHERE szenario.user_id = simulationuser.id AND szenario.title=?";

		PreparedStatement preparedStatement = ConnectionPool.getConnection()
				.prepareStatement(strSQL);

		preparedStatement.setString(1, name);
		ResultSet resultSet = preparedStatement.executeQuery();

		if (!resultSet.next()) {
			throw new IllegalStateException("Szenario could not be found: "
					+ name);
		}

		Szenario newSzenario = new Szenario(
				name,
				resultSet.getString("time_created"),
				resultSet.getString("user_name"));

		List<Conveyor> lstConveyor = loadConveyor(resultSet.getInt("id"));

		for (Conveyor newConveyor : lstConveyor) {
			newSzenario.addConveyor(newConveyor);
		}

		resultSet.close();
		preparedStatement.close();

		return newSzenario;
	}
	
	/**
	 * load conveyors from database into a list
	 * 
	 * @author Matthias
	 */
	private List<Conveyor> loadConveyor(int szenario_id) throws SQLException {
		List<Conveyor> lstConveyor = new ArrayList<Conveyor>();

		String strSQL = "SELECT type, pos_x, pos_y, direction " + "FROM "
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
			int direction = resultSet.getInt("direction");

			if (type.compareTo(ConveyorRamp.CONVEYOR_TYPE) == 0) {
				newConveyor = new ConveyorRamp(posX, posY);
			}
			else if (type.compareTo(ConveyorVehicle.CONVEYOR_TYPE) == 0) {
				newConveyor = new ConveyorVehicle(posX, posY);
			} 
			else if (type.compareTo(ConveyorWall.CONVEYOR_TYPE) == 0) {
				newConveyor = new ConveyorWall(posX, posY);
			}

			if (newConveyor != null) {
				newConveyor.rotate(direction);				
				lstConveyor.add(newConveyor);	
			}
		}

		resultSet.close();
		preparedStatement.close();

		return lstConveyor;
	}
}
