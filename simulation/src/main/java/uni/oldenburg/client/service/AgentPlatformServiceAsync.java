package uni.oldenburg.client.service;

import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.user.client.rpc.AsyncCallback;

/**
 * @author sijakubo Matthias
 */
public interface AgentPlatformServiceAsync {
	void startSimulation(Szenario szenario, AsyncCallback<Integer> async);
	void stopSimulation(AsyncCallback<Void> async);
}
