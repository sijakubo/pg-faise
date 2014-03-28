package uni.oldenburg.server.service;

import java.sql.SQLException;
import java.util.ArrayList;

import uni.oldenburg.client.service.SimulationService;
import uni.oldenburg.server.dao.SzenarioDao;
import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.user.server.rpc.RemoteServiceServlet;

@SuppressWarnings("serial")
public class SimulationServiceImpl extends RemoteServiceServlet implements
		SimulationService {
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
	public void saveSzenario(Szenario szenario, String selection) {
		SzenarioDao szenarioDao = new SzenarioDao();

		// Decide if a new szenario is inserted or an existing one is updated
		if (selection.equals("INSERT")) {
			try {
				szenarioDao.persistSzenario(szenario);
			} catch (SQLException e) {
				e.printStackTrace();
			}

		} else if (selection.equals("UPDATE")) {
			try {
				szenarioDao.updateSzenario(szenario);
			} catch (SQLException e) {
				e.printStackTrace();
			}
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

}
