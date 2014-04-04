package uni.oldenburg.server.dao;

import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.List;

import uni.oldenburg.server.databasehelper.ConnectionPool;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.ConveyorVehicle;
import uni.oldenburg.shared.model.ConveyorWall;
import uni.oldenburg.shared.model.Job;
import uni.oldenburg.shared.model.JobList;
import uni.oldenburg.shared.model.SimulationUser;
import uni.oldenburg.shared.model.Szenario;

public class JoblistDao {
	/**
	 * Checks wether a Joblist with the given title exists or not
	 * 
	 * @author Raschid
	 */
	public boolean checkIfJoblistExists(String titleJoblist)
			throws SQLException {
		String strSQL = "SELECT * FROM " + JobList.TABLE_NAME + " WHERE name=?";

		PreparedStatement preparedStatement = ConnectionPool.getConnection()
				.prepareStatement(strSQL);

		preparedStatement.setString(1, titleJoblist);

		ResultSet resultSet = preparedStatement.executeQuery();

		if (resultSet.next()) {
			return true;
		} else
			return false;

	}

	/**
	 * Returns the names of all Joblists so that they can be showed in the Popup
	 * for Joblist selection
	 * 
	 * @author Raschid
	 */
	public ArrayList<String> getJoblistTitles() throws SQLException {

		ArrayList<String> joblistTitles = new ArrayList<String>();

		String strSQL = "SELECT name FROM " + JobList.TABLE_NAME
				+ " ORDER BY name";

		PreparedStatement preparedStatement = ConnectionPool.getConnection()
				.prepareStatement(strSQL);

		ResultSet resultSet = preparedStatement.executeQuery();

		while (resultSet.next()) {

			String name = resultSet.getString("name");

			joblistTitles.add(name);

		}

		return joblistTitles;

	}

	/**
	 * Persists a Joblist into the Database by executing an Insert
	 * 
	 * @author Raschid
	 */
	public void persistJoblist(JobList joblist) throws SQLException {
		
		// delete Jobs of Joblist
		PreparedStatement prepStatement = ConnectionPool
				.getConnection()
				.prepareStatement(
						"DELETE FROM job WHERE joblist_id = (SELECT id from joblist WHERE name = ?)");

		prepStatement.setString(1, joblist.getName());
		prepStatement.executeUpdate();

		// delete current Joblist
		prepStatement = ConnectionPool.getConnection().prepareStatement(
				"DELETE FROM joblist WHERE name = ?");

		prepStatement.setString(1, joblist.getName());
		prepStatement.executeUpdate();

		prepStatement = ConnectionPool
				.getConnection()
				.prepareStatement(
						"INSERT INTO joblist ("
								+ "id, "
								+ "name) "
								+ "VALUES ("
								+ "(SELECT CASE WHEN MAX(ID) IS NULL THEN 0 ELSE MAX(ID)+1 END FROM joblist), "
								+ "?) ");

		prepStatement.setString(1, joblist.getName());

		prepStatement.executeUpdate();

		// Inserting the Jobs
		prepStatement = ConnectionPool
				.getConnection()
				.prepareStatement(
						"INSERT INTO "
								+ Job.TABLE_NAME
								+ " (id, joblist_id, package_id, destination_id, time_delay, type) "
								+ "VALUES "
								+ "(?, (SELECT id FROM joblist WHERE name = ?), ?, ?, ?, ?) ");

		// Persisting the Conveyors
		 ArrayList<Job> lstJobs = joblist.getJoblist();

		for (Job job : lstJobs) {
			prepStatement.setInt(1, job.getId());
			prepStatement.setString(2, joblist.getName());
			prepStatement.setInt(3, job.getPackageId());
			prepStatement.setInt(4, job.getDestinationId());
			prepStatement.setInt(5, job.getTimestamp());
			prepStatement.setInt(6, job.getType());
		
			prepStatement.executeUpdate();
		}
		
	}

	/**
	 * loads the Joblist by a name and intializes the Joblist and Jobs Objects
	 * with
	 * 
	 * @author Raschid
	 */
	public JobList loadJobList(String name) throws SQLException {
		String strSQL = "SELECT id, name " + "FROM " + JobList.TABLE_NAME + " "
				+ "WHERE name= ? ";

		PreparedStatement preparedStatement = ConnectionPool.getConnection()
				.prepareStatement(strSQL);

		preparedStatement.setString(1, name);
		ResultSet resultSet = preparedStatement.executeQuery();

		if (!resultSet.next()) {
			throw new IllegalStateException("Joblist could not be found: "
					+ name);
		}
		int idJoblist = resultSet.getInt("id");

		JobList jobList = new JobList(resultSet.getString("name"));

		List<Job> lstJob = loadJob(idJoblist);

		for (Job newJob : lstJob) {
			jobList.addJob(newJob);
		}

		resultSet.close();
		preparedStatement.close();

		return jobList;
	}

	/**
	 * load Jobs from database into a list
	 * 
	 * @author Raschid
	 */
	private List<Job> loadJob(int id_joblist) throws SQLException {
		List<Job> lstJobs = new ArrayList<Job>();

		String strSQL = "SELECT id, package_id, destination_id, time_delay, type "
				+ "FROM " + Job.TABLE_NAME + " " + "WHERE id = ?";

		PreparedStatement preparedStatement = ConnectionPool.getConnection()
				.prepareStatement(strSQL);

		preparedStatement.setInt(1, id_joblist);
		ResultSet resultSet = preparedStatement.executeQuery();

		while (resultSet.next()) {
			Job job = null;

			int timestamp = resultSet.getInt("timestamp");
			int destinationId = resultSet.getInt("destinationId");
			int packageId = resultSet.getInt("packageId");
			int type=resultSet.getInt("type");

			job = new Job(type,timestamp,packageId, destinationId);

			if (job != null) {

				lstJobs.add(job);
			}
		}

		resultSet.close();
		preparedStatement.close();

		return lstJobs;
	}
}
