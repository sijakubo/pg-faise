package uni.oldenburg.server.agent;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

@SuppressWarnings("serial")
public class RampRoutingAgent extends Agent {
	public final static String NAME = "RampRoutingAgent";
	
	private int conveyorID = 0;
	private int szenarioID = 0;
	
	private Logger logger = Logger.getLogger(RampRoutingAgent.class);
	
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
		
		addBehaviour(new AssignVehicleForPackageBehaviour());
		addBehaviour(new StartAuctionBehaviour());
		addBehaviour(new ReceiveEstimationBehaviour());
		
		String nickname = AgentHelper.getUniqueNickname(RampRoutingAgent.NAME, conveyorID, szenarioID);		
		AgentHelper.registerAgent(szenarioID, this, nickname);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}

	public int getConveyorID() {
		return this.conveyorID;
	}

	public int getSzenarioID() {
		return this.szenarioID;
	}
	
	/**
	 * @author Christopher
	 */
	private class AssignVehicleForPackageBehaviour extends OneShotBehaviour {
		public void action() {
			// send message
			ACLMessage msg = new ACLMessage(MessageType.ASSIGN_VEHICLE_FOR_PACKAGE);
			
			String sourceID = "" + ((RampRoutingAgent)myAgent).getConveyorID();
			String destinationID = "tbd";
			String botID = "tbd";
			String packageID = "tbd";

			msg.addUserDefinedParameter("sourceID", sourceID);
			msg.addUserDefinedParameter("destinationID", destinationID);
			msg.addUserDefinedParameter("botID", botID);
			msg.addUserDefinedParameter("packageID", packageID);
			
			AgentHelper.addReceivers(msg, myAgent, ((RampRoutingAgent)myAgent).getSzenarioID());
			
			if(Debugging.showAuctionMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " sent ASSIGN_VEHICLE_FOR_TRANSPORT message");

			send(msg);
		}
	}

	/**
	 * @author Christopher
	 */
	private class StartAuctionBehaviour extends OneShotBehaviour {
		public void action() {
			// send message
			ACLMessage msg = new ACLMessage(MessageType.START_AUCTION);
			
			String auctionID = "tbd";
			String sourceID = "" + ((RampRoutingAgent)myAgent).getConveyorID();
			String destinationID = "tbd";
			
			msg.addUserDefinedParameter("auctionID", auctionID);
			msg.addUserDefinedParameter("sourceID", sourceID);
			msg.addUserDefinedParameter("destinationID", destinationID);
			
			AgentHelper.addReceivers(msg, myAgent, ((RampRoutingAgent)myAgent).getSzenarioID());
			
			if(Debugging.showAuctionMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " sent START_AUCTION message");
			
			send(msg);
		}
	}
	
	/**
	 * @author Christopher
	 */
	private class ReceiveEstimationBehaviour extends Behaviour {

		public void action() {
			
			int auctionID;
			int estimation;

			// wait for message
			MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.SEND_ESTIMATION);
			ACLMessage msg = myAgent.blockingReceive(mt);
			try {
				auctionID = Integer.valueOf(msg.getUserDefinedParameter("auctionID"));
			} catch(NumberFormatException e) {
				auctionID = -1;
			}
			try {
				estimation = Integer.valueOf(msg.getUserDefinedParameter("estimation"));
			} catch(NumberFormatException e) {
				estimation = -1;
			}
			
			if(Debugging.showAuctionMessages) {
				logger.log(Level.INFO, myAgent.getLocalName() + " received SEND_ESTIMATION message #" + auctionID + ": " + estimation);
			}
			
		}

		public boolean done() {
			return false;
		}
	}
}
