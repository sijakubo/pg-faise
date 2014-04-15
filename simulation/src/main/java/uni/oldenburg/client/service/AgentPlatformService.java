package uni.oldenburg.client.service;

import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.user.client.rpc.RemoteService;
import com.google.gwt.user.client.rpc.RemoteServiceRelativePath;

/**
 * @author sijakubo Matthias
 */
@RemoteServiceRelativePath("agentPlatformService")
public interface AgentPlatformService extends RemoteService {
	int startSimulation(Szenario szenario);
	void stopSimulation();
}
