package uni.oldenburg.client.service;

import java.util.ArrayList;

import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.user.client.rpc.RemoteService;
import com.google.gwt.user.client.rpc.RemoteServiceRelativePath;

@RemoteServiceRelativePath("simulationsService")
public interface SimulationService extends RemoteService {
	Szenario loadSzenario(String name);
	ArrayList<String> getScenarioTitles();
	void saveSzenario(Szenario szenario,String selection);
	boolean checkIfTitleExists(String title);
}
