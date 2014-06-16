package uni.oldenburg.server.dao;

import java.sql.SQLException;

import junit.framework.Assert;

import org.junit.Test;

import uni.oldenburg.shared.model.JobList;
import uni.oldenburg.shared.model.Szenario;

public class JobListDaoTest {
	@Test
	public void testLoadJobList() {
		JoblistDao jobListDao = new JoblistDao();

		try {
			JobList test = jobListDao.loadJobList("Test");
			Assert.assertNotNull(test);
			Assert.assertTrue(test.getName().equals("Test"));
		} catch (SQLException e) {
			e.printStackTrace();
			Assert.fail();
		}
	}

	@Test
	public void testPersistJobList() {
		JoblistDao jobListDao = new JoblistDao();

		try {
			// Dummy Joblist is created and persisted
			JobList list = new JobList("new Joblist");
			jobListDao.persistJoblist(list);
			// Then load it to check if it was persisted
			JobList testList = jobListDao.loadJobList("new Joblist");
			Assert.assertNotNull(testList);
			Assert.assertTrue(testList.getName().equals("new Joblist"));
			// Remove the Joblist, because the Database should not be changed by
			// a test
			jobListDao.removeJobList("new Joblist");
		} catch (SQLException e) {
			e.printStackTrace();
			Assert.fail();
		}
	}

	@Test
	public void testCheckIfJobListExists() {
		JoblistDao jobListDao = new JoblistDao();

		try {
			// Assert that Method returns true for an existing Joblist and
			// false for a not existing Joblist
			Assert.assertTrue(jobListDao.checkIfJoblistExists("Test"));
			Assert.assertFalse(jobListDao.checkIfJoblistExists("Testing456789"));
		} catch (SQLException e) {
			e.printStackTrace();
			Assert.fail();
		}
	}
}
