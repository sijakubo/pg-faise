package uni.oldenburg.server.service;

import com.google.gwt.user.server.rpc.RemoteServiceServlet;
import jade.core.ProfileImpl;
import jade.core.Runtime;
import jade.wrapper.ContainerController;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import uni.oldenburg.client.service.AgentPlatformService;

public class AgentPlatformServiceImpl extends RemoteServiceServlet implements AgentPlatformService {
   Logger logger = Logger.getLogger(AgentPlatformServiceImpl.class);

   /**
    * @author sijakubo
    */
   public void startSimulation() {
      startAgentPlatform();
      logger.log(Level.INFO, "Agent Platform started");
   }

   /**
    * @author sijakubo
    */
   private void startAgentPlatform() {
      Runtime runtimeInstance = Runtime.instance();
      ContainerController container = runtimeInstance.createMainContainer(new ProfileImpl());
      //container.createNewAgent("admin", "uez2.AdminAgent", null)
   }
}
