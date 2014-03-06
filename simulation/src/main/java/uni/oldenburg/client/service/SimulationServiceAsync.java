package uni.oldenburg.client.service;

import uni.oldenburg.shared.model.Szenario;
import com.google.gwt.user.client.rpc.AsyncCallback;

public interface SimulationServiceAsync extends ServiceAsync {
	void loadSzenario(String name, AsyncCallback<Szenario> asyncCallback);
}

