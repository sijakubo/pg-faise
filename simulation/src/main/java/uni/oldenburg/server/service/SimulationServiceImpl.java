package uni.oldenburg.server.service;

import java.sql.SQLException;

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
}
