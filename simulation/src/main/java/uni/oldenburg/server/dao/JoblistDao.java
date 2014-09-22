package uni.oldenburg.server.dao;

import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.List;

import uni.oldenburg.server.databasehelper.ConnectionPool;
import uni.oldenburg.shared.model.Job;
import uni.oldenburg.shared.model.JobList;

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
	 * Removes a Joblist by its name
	 * 
	 * @author Raschid 
	 * @throws SQLException 
	 */
	public void removeJobList(String name) throws SQLException{
		// delete current Joblist
		PreparedStatement prepStatement = ConnectionPool.getConnection().prepareStatement(
						"DELETE FROM joblist WHERE name = ?");

		prepStatement.setString(1, name);
		prepStatement.executeUpdate();
	}

	/**
	 * Persists a Joblist into the Database by executing an Insert
	 * 
	 * @author Raschid Matthias
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
								+ " (id, joblist_id, destination_id, time_delay) "
								+ "VALUES ("
								+ "(SELECT CASE WHEN MAX(ID) IS NULL THEN 0 ELSE MAX(ID)+1 END FROM job) , (SELECT id FROM joblist WHERE name = ?), ?, ?) ");

		// Persisting the Conveyors
		List<Job> lstJobs = joblist.getJoblist();

		for (Job job : lstJobs) {
			prepStatement.setString(1, joblist.getName());
			prepStatement.setInt(2, job.getDestinationId());
			prepStatement.setInt(3, job.getTimestamp());
		
			prepStatement.executeUpdate();
		}
		
	}

	/**
	 * loads the Joblist by a name and intializes the Joblist and Jobs Objects
	 * with
	 * 
	 * @author Raschid Matthias
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
        JobList.lastTimestamp=0;
        Job.idCounterPacket=0;
		List<Job> lstJob = loadJobs(idJoblist, jobList);

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
	private List<Job> loadJobs(int id_joblist, JobList jobList) throws SQLException {
		List<Job> lstJobs = new ArrayList<Job>();

		String strSQL = "SELECT destination_id, time_delay "
				+ "FROM " + Job.TABLE_NAME + " " + "WHERE joblist_id = ?";

		PreparedStatement preparedStatement = ConnectionPool.getConnection().prepareStatement(strSQL);

		preparedStatement.setInt(1, id_joblist);
		ResultSet resultSet = preparedStatement.executeQuery();

		while (resultSet.next()) {
			int timestamp = resultSet.getInt("time_delay");
			int destinationId = resultSet.getInt("destination_id");

			Job job = new Job(timestamp, destinationId, jobList);

			if (job != null) {
				lstJobs.add(job);
			}
		}

		resultSet.close();
		preparedStatement.close();

		return lstJobs;
	}
}
