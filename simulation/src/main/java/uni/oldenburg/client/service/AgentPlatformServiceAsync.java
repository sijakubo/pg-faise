package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.AsyncCallback;

public interface AgentPlatformServiceAsync {
   void startSimulation(AsyncCallback<Void> async);
}
