package uni.oldenburg.server.service;

import com.google.gwt.user.server.rpc.RemoteServiceServlet;
import jade.core.Agent;
import jade.core.ProfileImpl;
import jade.core.Runtime;
import jade.wrapper.ContainerController;
import jade.wrapper.StaleProxyException;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import uni.oldenburg.client.service.AgentPlatformService;

public class AgentPlatformServiceImpl extends RemoteServiceServlet implements AgentPlatformService {
   private Logger logger = Logger.getLogger(AgentPlatformServiceImpl.class);
   private ContainerController container;

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
      container = runtimeInstance.createMainContainer(new ProfileImpl());
      //container.createNewAgent("admin", "uez2.AdminAgent", null)
   }

   /**
    * @author sijakubo
    */
   public void addAgentToSimulationPlatform(Agent agent, String nickname) throws StaleProxyException {
      container.acceptNewAgent(nickname, agent);
   }
}
