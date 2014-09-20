package uni.oldenburg.client.service;

import uni.oldenburg.shared.model.Job;
import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.user.client.rpc.RemoteService;
import com.google.gwt.user.client.rpc.RemoteServiceRelativePath;

/**
 * @author sijakubo Matthias Raschid
 */
@RemoteServiceRelativePath("agentPlatformService")
public interface AgentPlatformService extends RemoteService {
	int startSimulation(Szenario szenario);
	void stopSimulation();
	void addJob(int szenarioID, Job myJob);
}
