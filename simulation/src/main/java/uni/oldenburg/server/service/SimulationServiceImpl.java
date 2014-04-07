package uni.oldenburg.server.service;

import java.sql.SQLException;
import java.util.ArrayList;

import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpSession;

import uni.oldenburg.client.service.SimulationService;
import uni.oldenburg.server.dao.JoblistDao;
import uni.oldenburg.server.dao.SzenarioDao;
import uni.oldenburg.shared.model.JobList;
import uni.oldenburg.shared.model.SimulationUser;
import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.user.server.rpc.RemoteServiceServlet;

@SuppressWarnings("serial")
public class SimulationServiceImpl extends RemoteServiceServlet implements SimulationService {
	/**
	 * load szenario based on unique name
	 * 
	 * @author Matthias
	 */
	public Szenario loadSzenario(String name) {
		SzenarioDao szenarioDao = new SzenarioDao();
		Szenario newSzenario = null;

		try {
			newSzenario = szenarioDao.loadSzenario(name);
		} catch (SQLException e) {
			e.printStackTrace();
		}

		return newSzenario;
	}

	/**
	 * Method gets the Name of the ScenarioTitles in order to present it within
	 * the Popup
	 * 
	 * @author Raschid
	 */
	public ArrayList<String> getScenarioTitles() {
		SzenarioDao szenarioDao = new SzenarioDao();
		ArrayList<String> scenarioTitles = null;

		try {
			scenarioTitles = szenarioDao.getSzenarioTitles();
		} catch (SQLException e) {
			e.printStackTrace();
		}

		return scenarioTitles;
	}

	/**
	 * Saves the Szenario, which comes from the Client, into the Database The
	 * selection String decides wether an existing szenario is updated or a new
	 * one is inserted
	 * 
	 * @author Raschid
	 */
	public void saveSzenario(Szenario szenario) {
		SzenarioDao szenarioDao = new SzenarioDao();
		
		try {
			szenarioDao.persistSzenario(szenario);
		} catch (SQLException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Checks if the Szenario exists, by calling the corresponding Database
	 * method
	 * 
	 * @author Raschid
	 */
	public boolean checkIfTitleExists(String title) {
		SzenarioDao szenarioDao = new SzenarioDao();
		boolean result = false;
		try {
			result = szenarioDao.checkIfTitleExists(title);
		} catch (SQLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return result;
	}


   public String getUserName() {
      HttpServletRequest httpServletRequest = this.getThreadLocalRequest();
      HttpSession session = httpServletRequest.getSession(true);
      SimulationUser simUser = (SimulationUser) session.getAttribute("user");

      String username;
      if (simUser != null) {
         username = simUser.getName();
      } else {
         username = "";
      }
      return username;
   }

	
	/**
	 * load Joblist based on unique name
	 * 
	 * @author Raschid
	 */
	public JobList loadJoblist(String name) {
		JoblistDao joblistDao = new JoblistDao();
		JobList newJoblist = null;

		try {
			newJoblist = joblistDao.loadJobList(name);
		} catch (SQLException e) {
			e.printStackTrace();
		}

		return newJoblist;
	}

	/**
	 * Method gets the Name of the Jonlist in order to present it within
	 * the Popup
	 * 
	 * @author Raschid
	 */
	public ArrayList<String> getJoblistTitles() {
		JoblistDao joblistDao = new JoblistDao();
		ArrayList<String> joblistTitles = null;

		try {
			joblistTitles = joblistDao.getJoblistTitles();
		} catch (SQLException e) {
			e.printStackTrace();
		}

		return joblistTitles;
	}

	/**
	 * Saves the Joblist, which comes from the Client, into the Database. 
	 * 
	 * @author Raschid
	 */
	public void saveJoblist(JobList list) {
		JoblistDao joblistDao = new JoblistDao();
		
		try {
			joblistDao.persistJoblist(list);
		} catch (SQLException e) {
			e.printStackTrace();
			System.out.println("Save Joblist");
		}
	}

	/**
	 * Checks if a Joblist exists, by calling the corresponding Database
	 * method
	 * 
	 * @author Raschid
	 */
	public boolean checkIfJobListExists(String title) {
		JoblistDao joblistDao = new JoblistDao();
		boolean result = false;
		try {
			result = joblistDao.checkIfJoblistExists(title);
		} catch (SQLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return result;
	}
	
	
}
