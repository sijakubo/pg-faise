package uni.oldenburg.client.service;

import uni.oldenburg.shared.model.Job;
import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.user.client.rpc.AsyncCallback;

/**
 * @author sijakubo Matthias
 */
public interface AgentPlatformServiceAsync {
	void startSimulation(Szenario szenario, AsyncCallback<Integer> async);
	void stopSimulation(AsyncCallback<Void> async);
	void addJob(int szenarioID, Job myJob, AsyncCallback<Void> async);
	void setSimulationSpeed(int szenarioID, int value, AsyncCallback<Void> async);
}
