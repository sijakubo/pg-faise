package uni.oldenburg.client.service;

import uni.oldenburg.shared.model.Szenario;
import com.google.gwt.user.client.rpc.RemoteService;
import com.google.gwt.user.client.rpc.RemoteServiceRelativePath;

@RemoteServiceRelativePath("simulationsService")
public interface SimulationService extends RemoteService {
	Szenario loadSzenario(String name);
}
