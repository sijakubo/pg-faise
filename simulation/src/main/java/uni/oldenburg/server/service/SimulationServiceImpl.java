package uni.oldenburg.server.service;

import java.sql.SQLException;
import java.util.ArrayList;

import uni.oldenburg.client.service.SimulationService;
import uni.oldenburg.server.dao.SzenarioDao;
import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.user.server.rpc.RemoteServiceServlet;


@SuppressWarnings("serial")
public class SimulationServiceImpl extends RemoteServiceServlet implements SimulationService {
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

	//Method gets the Name of the ScenarioTitles in order to present it within the Popup
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
	
	
	
	
}
