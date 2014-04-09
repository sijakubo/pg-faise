package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.rpc.RemoteService;
import com.google.gwt.user.client.rpc.RemoteServiceRelativePath;

public interface AgentPlatformServiceAsync {
   void startSimulation(AsyncCallback<Void> async);
}
