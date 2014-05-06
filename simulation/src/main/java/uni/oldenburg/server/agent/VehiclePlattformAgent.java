package uni.oldenburg.server.agent;

import java.io.IOException;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.data.PackageData;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

@SuppressWarnings("serial")
public class VehiclePlattformAgent extends Agent {
	public final static String NAME = "VehiclePlattformAgent";
	
	private int conveyorID = 0;
	private int szenarioID = 0;
	
	private Logger logger = Logger.getLogger(VehiclePlattformAgent.class);
	
	/**
     * @author Matthias
     */	
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			szenarioID = (Integer) args[0];
			
			Conveyor myConveyor = (Conveyor) args[1];
			conveyorID = myConveyor.getID();
		}
		
		//addBehaviour(new IsFreeForTransportBehaviour());
		
		String nickname = AgentHelper.getUniqueNickname(VehiclePlattformAgent.NAME, conveyorID, szenarioID);		
		AgentHelper.registerAgent(szenarioID, this, nickname);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
		
		addBehaviour(new GetPackageFromSourceBehaviour(MessageTemplate.MatchPerformative(MessageType.GET_PACKAGE_FROM_SOURCE)));
		addBehaviour(new BotGoToDestinationBehaviour(MessageTemplate.MatchPerformative(MessageType.BOT_GO_TO_DESTINATION)));
		
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * Behaviour should receive the request from its RoutingAgent and start the Communication with the Source Ramp, where the Package
	 * lies
	 * @author Raschid
	 */
	private class GetPackageFromSourceBehaviour extends
			CyclicReceiverBehaviour {

		protected GetPackageFromSourceBehaviour(MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			VehiclePlattformAgent currentAgent = (VehiclePlattformAgent) myAgent;
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- GET_PACKAGE_FROM_SOURCE");
			
			//Tell the Rampplattformagent that he wants to have the Package and which Package he wants to have
			ACLMessage msgPackage = new ACLMessage(MessageType.GIVE_PACKAGE);
			msgPackage.addUserDefinedParameter("packageID", msg.getUserDefinedParameter("packageID"));
			AgentHelper.addReceiver(msgPackage, myAgent,RampPlattformAgent.NAME, Integer.parseInt(msg.getUserDefinedParameter("sourceID")),currentAgent.szenarioID);
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " -> GIVE_ME_PACKAGE");
			
			send(msgPackage);
			
			
						
            //Receive Message from Plattformagent and Get the Package and initialize the Packageagent with the Package
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- ANSWER_BOT");
			MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.PACKAGE_REMOVED);
			ACLMessage msgGetAnswer = myAgent.blockingReceive(mt);
			
			
			ACLMessage addPackage = new ACLMessage(MessageType.BOT_ADD_PACKAGE);
			addPackage.setContentObject(msgGetAnswer.getContentObject());
			AgentHelper.addReceiver(addPackage, currentAgent,PackageAgent.NAME, currentAgent.conveyorID, currentAgent.szenarioID);
			
			
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " -> BOT_ADD_PACKAGE");
			
			send(addPackage);
			
			//Go to destination (Selfmessage)
			ACLMessage goDestination = new ACLMessage(MessageType.BOT_GO_TO_DESTINATION);
			goDestination.addUserDefinedParameter("sourceID", msg.getUserDefinedParameter("sourceID"));
			AgentHelper.addReceiver(addPackage, currentAgent,VehiclePlattformAgent.NAME, currentAgent.conveyorID, currentAgent.szenarioID);
			
			send(goDestination);
			
		}

	}
	
	
	/**
	 * Behaviour should ask the Destination Ramp to take the Package
	 * @author Raschid
	 */
	private class BotGoToDestinationBehaviour extends
			CyclicReceiverBehaviour {

		protected BotGoToDestinationBehaviour(MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			VehiclePlattformAgent currentAgent = (VehiclePlattformAgent) myAgent;
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- GET_PACKAGE_FROM_SOURCE");
			
			//Tell the Rampplattformagent that he wants to have the Package and which Package he wants to have
			ACLMessage msgPackage = new ACLMessage(MessageType.GIVE_PACKAGE);
			msgPackage.addUserDefinedParameter("packageID", msg.getUserDefinedParameter("packageID"));
			AgentHelper.addReceiver(msgPackage, myAgent,RampPlattformAgent.NAME, Integer.parseInt(msg.getUserDefinedParameter("sourceID")),currentAgent.szenarioID);
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " -> GIVE_ME_PACKAGE");
			
			send(msgPackage);
			
			
						
            //Receive Message from Plattformagent and Get the Package and initialize the Packageagent with the Package
			MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.PACKAGE_REMOVED);
			ACLMessage msgGetAnswer = myAgent.blockingReceive(mt);
			
			ACLMessage addPackage = new ACLMessage(MessageType.BOT_ADD_PACKAGE);
			addPackage.setContentObject(msgGetAnswer.getContentObject());
			addPackage.addReceiver(msg.getSender());
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- ANSWER_BOT");
			
			send(addPackage);
			
			//Go to destination (Selfmessage)
			
			
			
			
		}

	}
	
	
	
}
