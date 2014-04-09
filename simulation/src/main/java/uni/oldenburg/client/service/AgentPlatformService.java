package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.RemoteService;
import com.google.gwt.user.client.rpc.RemoteServiceRelativePath;

@RemoteServiceRelativePath("agentPlatformService")
public interface AgentPlatformService extends RemoteService {

   void startSimulation();
}
