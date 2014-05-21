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
import uni.oldenburg.shared.model.Szenario;

@SuppressWarnings("serial")
public class VehiclePlattformAgent extends Agent {
	public final static String NAME = "VehiclePlattformAgent";
	
	private int conveyorID = 0;
	private Szenario szenario;
	
	private Logger logger = Logger.getLogger(VehiclePlattformAgent.class);
	
	/**
     * @author Matthias
     */	
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			szenario = (Szenario) args[0];
			
			Conveyor myConveyor = (Conveyor) args[1];
			conveyorID = myConveyor.getID();
		}
		
		//addBehaviour(new IsFreeForTransportBehaviour());
		
		String nickname = AgentHelper.getUniqueNickname(VehiclePlattformAgent.NAME, conveyorID, szenario.getId());		
		AgentHelper.registerAgent(szenario.getId(), this, nickname);
		
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
	 * Got message:
	 * 		VehicleRoutingAgent: AssignVehicleForPackageBehaviour 
	 *      RampPlattformAgent: GivePackageBehaviour     
	 * Send message:
	 *      RampPlattformAgent: GivePackageBehaviour
	 * 		PackageAgent: BotAddPackageBehaviour
	 *      VehiclePlattformAgent: BotGoToDestinationBehaviour 
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
			AgentHelper.addReceiver(msgPackage, myAgent,RampPlattformAgent.NAME, Integer.parseInt(msg.getUserDefinedParameter("sourceID")),currentAgent.szenario.getId());
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " -> GIVE_PACKAGE");
			
			send(msgPackage);
			
			
						
            //Receive Message from Plattformagent and Get the Package and initialize the Packageagent with the Package
			
			MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.ANSWER_BOT);
			ACLMessage msgGetAnswer = myAgent.blockingReceive(mt);
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- ANSWER_BOT");
			
			ACLMessage addPackage = new ACLMessage(MessageType.BOT_ADD_PACKAGE);
			addPackage.setContentObject(msgGetAnswer.getContentObject());
			AgentHelper.addReceiver(addPackage, currentAgent,PackageAgent.NAME, currentAgent.conveyorID, currentAgent.szenario.getId());
			
			
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " -> BOT_ADD_PACKAGE");
			
			send(addPackage);
			
			//Go to destination (Selfmessage)
			ACLMessage goDestination = new ACLMessage(MessageType.BOT_GO_TO_DESTINATION);
			goDestination.addUserDefinedParameter("destinationID", msg.getUserDefinedParameter("destinationID"));
			AgentHelper.addReceiver(goDestination, currentAgent,VehiclePlattformAgent.NAME, currentAgent.conveyorID, currentAgent.szenario.getId());
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " -> BOT_GO_TO_DESTINATION");
			
			send(goDestination);
			
		}

	}
	
	
	/**
	 * Got message:
	 *      VehiclePlattformAgent: GetPackageFromSourceBehaviour
	 *      RampPlattformAgent: ReceivePackageBehaviour
	 *      PackageAgent: BotRemovePackageBehaviour     
	 * Send message:
	 *      PackageAgent: BotRemovePackageBehaviour 
	 *      RampPlattformAgent: ReceivePackageBehaviour
	 *      VehicleRoutingAgent: SetBotUnreservedBehaviour
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
			//Receive the Message from itself and start asking the destination Ramp
			VehiclePlattformAgent currentAgent = (VehiclePlattformAgent) myAgent;
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- BOT_GO_TO_DESTINATION");
			
			ACLMessage targetAchieved = new ACLMessage(MessageType.BOT_TARGET_ACHIEVED );
			AgentHelper.addReceiver(targetAchieved, currentAgent,RampPlattformAgent.NAME, Integer.parseInt(msg.getUserDefinedParameter("destinationID")), currentAgent.szenario.getId());
			send(targetAchieved);
			
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " -> BOT_TARGET_ACHIEVED");
			
			
			//Receive Answer from Plattformagent
			MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.CAN_TAKE_PACKAGE);
			ACLMessage msgGetAnswer = myAgent.blockingReceive(mt);
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- CAN_TAKE_PACKAGE");
			
			//Send Message to Packageagent, that he removes and overgives the package
			ACLMessage removePackage = new ACLMessage(MessageType.BOT_REMOVE_PACKAGE);
			AgentHelper.addReceiver(removePackage, currentAgent,PackageAgent.NAME, currentAgent.conveyorID, currentAgent.szenario.getId());
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " -> BOT_REMOVE_PACKAGE");
			send(removePackage);
			
			
			//Receive the Package from Packageagent
			MessageTemplate mtP = MessageTemplate.MatchPerformative(MessageType.BOT_REMOVED_PACKAGE);
			ACLMessage msgGetAnswerFromP = myAgent.blockingReceive(mtP);
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- BOT_REMOVED_PACKAGE");
			
			//Tell the Ramp to Take the Package
			ACLMessage takePackage = new ACLMessage(MessageType.RAMP_TAKE_PACKAGE);
			takePackage.setContentObject(msgGetAnswerFromP.getContentObject());
			AgentHelper.addReceiver(takePackage, currentAgent,RampPlattformAgent.NAME,Integer.parseInt(msg.getUserDefinedParameter("destinationID")), currentAgent.szenario.getId());
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " -> RAMP_TAKE_PACKAGE");
			
			send(takePackage);
			
			//Tell the Bot to set its status unreserved
			ACLMessage unreserved = new ACLMessage(MessageType.SET_BOT_UNRESERVED);
			
			AgentHelper.addReceiver(unreserved, currentAgent,VehicleRoutingAgent.NAME,currentAgent.conveyorID, currentAgent.szenario.getId());
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " -> SET_BOT_UNRESERVED");
			
			send(unreserved);
			
			
		}

	}
	
	
	
}
