package uni.oldenburg.server.service;

import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import com.google.gwt.user.server.rpc.RemoteServiceServlet;

import de.novanic.eventservice.client.event.domain.DomainFactory;
import jade.core.Agent;
import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.core.Runtime;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.ContainerController;
import jade.wrapper.StaleProxyException;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.client.presenter.MainFramePresenter;
import uni.oldenburg.client.service.AgentPlatformService;
import uni.oldenburg.server.agent.JobAgent;
import uni.oldenburg.server.agent.PackageAgent;
import uni.oldenburg.server.agent.RampOrderAgent;
import uni.oldenburg.server.agent.RampPlattformAgent;
import uni.oldenburg.server.agent.RampRoutingAgent;
import uni.oldenburg.server.agent.VehiclePlattformAgent;
import uni.oldenburg.server.agent.VehicleRoutingAgent;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.ConveyorVehicle;
import uni.oldenburg.shared.model.Job;
import uni.oldenburg.shared.model.Szenario;
import uni.oldenburg.shared.model.event.SimStoppedEvent;

@SuppressWarnings("serial")
public class AgentPlatformServiceImpl extends RemoteServiceServlet implements AgentPlatformService {
   private Logger logger = Logger.getLogger(AgentPlatformServiceImpl.class);
   private ContainerController container = null;
   private List<AgentController> lstAgent = null;
   private static int szenarioID = 0;
   Map<Integer, JobAgent> mapJobAgent = new HashMap<Integer, JobAgent>();

   /**
    * puts job from client into simulation
    * 
    * @author Matthias
    */
   public void addJob(int szenarioID, Job myJob) {	   
	   Agent myAgent = mapJobAgent.get(szenarioID);
	   
	   ACLMessage msg = new ACLMessage(MessageType.ASSIGN_JOB);			
	   msg.addReceiver(myAgent.getAID());
	   
	   try {
		   msg.setContentObject(myJob);
		   myAgent.send(msg);   		
	   }
	   catch (IOException e) {
		   e.printStackTrace();
	   }
   }
   
   /**
    * start simulation
    * 
    * @author sijakubo Matthias
    */
   public int startSimulation(Szenario szenario) {
	   // async function call and static variable, so remember,
	   // in case someone else starts before function is finished
	   int mySzenarioID = ++szenarioID;
	   
	   Object[] argsJobAgent = new Object[1];
	   argsJobAgent[0] = mySzenarioID;
	   
	   List<Conveyor> lstConveyor = szenario.getConveyorList();
	   
	   if (lstConveyor.size() < 1)
		   return -1;
	   
	   startAgentPlatform(szenario);
	   
	   addAgentToSimulation(0, mySzenarioID, argsJobAgent, JobAgent.NAME, new JobAgent());
	   
	   for (Conveyor myConveyor : lstConveyor) {
		   Object[] argsAgent = new Object[2];
		   argsAgent[0] = mySzenarioID;
		   argsAgent[1] = myConveyor;
		   
		   int id = myConveyor.getID();		   
		   
		   if (myConveyor instanceof ConveyorRamp) {
			   addAgentToSimulation(id, mySzenarioID, argsAgent, PackageAgent.NAME			, new PackageAgent());
			   addAgentToSimulation(id, mySzenarioID, argsAgent, RampOrderAgent.NAME		, new RampOrderAgent());
			   addAgentToSimulation(id, mySzenarioID, argsAgent, RampPlattformAgent.NAME 	, new RampPlattformAgent());
			   addAgentToSimulation(id, mySzenarioID, argsAgent, RampRoutingAgent.NAME		, new RampRoutingAgent());
		   } else if (myConveyor instanceof ConveyorVehicle) {
			   addAgentToSimulation(id, mySzenarioID, argsAgent, PackageAgent.NAME			, new PackageAgent());
			   addAgentToSimulation(id, mySzenarioID, argsAgent, VehiclePlattformAgent.NAME	, new VehiclePlattformAgent());
			   addAgentToSimulation(id, mySzenarioID, argsAgent, VehicleRoutingAgent.NAME	, new VehicleRoutingAgent());
		   }
	   }
	   
	   startAgents();
	   
	   return mySzenarioID;
   }
   
   /**
    * stop simulation
    * 
    * @author Matthias
    */
   public void stopSimulation() {
	   if (lstAgent != null)
		   lstAgent.clear();
	   
	   lstAgent = null;
	   
	   try {
		   if (container != null)
			   container.kill();
	   }
	   catch (StaleProxyException e) {
		   e.printStackTrace();
	   }
	   
		// fire SimStoppedEvent
		new RemoteEventService().addEvent(DomainFactory.getDomain(MainFramePresenter.DOMAIN_NAME), new SimStoppedEvent());
	   
	   container = null;
   }
   
   /**
    * start all created agents
    * 
    * @author Matthias
    */
   private void startAgents() {
	   for (AgentController myAgentController : lstAgent) {
		   if (myAgentController != null) {
			   try {
				   myAgentController.start();
			   }
			   catch (StaleProxyException e) {
				   e.printStackTrace();
			   }   
		   }
	   }
   }
   
   /**
    * @author sijakubo Matthias
    */
   private void startAgentPlatform(Szenario szenario) {
	   if (container == null) {
		   Profile profile = new ProfileImpl();
		   // can be read from every assigned agent
		   profile.setParameter("width",  "" + szenario.getWidth());
		   profile.setParameter("height", "" + szenario.getHeight());
		   
		   logger.log(Level.INFO, "Agent Platform started");
		   Runtime runtimeInstance = Runtime.instance();
		   container = runtimeInstance.createMainContainer(profile);
		   
		   lstAgent = new LinkedList<AgentController>();
	   }
   }

   /**
    * Erstellt den JADE-Runtime Container. An den Container können Agenten angemeldet werden,
    * die miteinander innerhalb des Containers kommunizieren können.
    * 
    * @author sijakubo Matthias
    */
   private void addAgentToSimulation(int conveyorID, int szenarioID, Object[] args, String nickname, Agent myAgent) {
	   AgentController myAgentController = null;
	   
	   try {
		   nickname = AgentHelper.getUniqueNickname(nickname, conveyorID, szenarioID);
		   
		   myAgent.setArguments(args);   
		   
		   myAgentController = container.acceptNewAgent(nickname, myAgent);
		   lstAgent.add(myAgentController);
		   
		   if (myAgent instanceof JobAgent) {
			   mapJobAgent.put(szenarioID, (JobAgent)myAgent);   
		   }
	   } 
	   catch (StaleProxyException e) {
		   e.printStackTrace();
	   }
   }
}
