package uni.oldenburg.server.agent;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

@SuppressWarnings("serial")
public class VehicleRoutingAgent extends Agent {
	public final static String NAME = "VehicleRoutingAgent";
	
	private int conveyorID = 0;
	private int szenarioID = 0;
	
	private Logger logger = Logger.getLogger(VehicleRoutingAgent.class);
	
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
		addBehaviour(new InitializePacketAgentBehaviour());
		addBehaviour(new IsFreeForTransportBehaviour());
		
		String nickname = AgentHelper.getUniqueNickname(VehicleRoutingAgent.NAME, conveyorID, szenarioID);		
		AgentHelper.registerAgent(szenarioID, this, nickname);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}

	private class AssignVehicleForPackageBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
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
	private class StartAuctionBehaviour extends Behaviour {

		public void action() {

			int auctionID;
			int sourceID;
			int destinationID;
			
			// wait for message
			MessageTemplate mt = MessageTemplate.MatchPerformative(MessageType.START_AUCTION);
			ACLMessage msgIn = myAgent.blockingReceive(mt);

			try {
				auctionID = Integer.valueOf(msgIn.getUserDefinedParameter("auctionID"));
			} catch(NumberFormatException e) {
				auctionID = -1;
			}
			try {
				sourceID = Integer.valueOf(msgIn.getUserDefinedParameter("sourceID"));
			} catch(NumberFormatException e) {
				sourceID = -1;
			}
			try {
				destinationID = Integer.valueOf(msgIn.getUserDefinedParameter("destinationID"));
			} catch(NumberFormatException e) {
				destinationID = -1;
			}
			
			if(Debugging.showAuctionMessages) {
				logger.log(Level.INFO, myAgent.getLocalName() + " received START_AUCTION message #" + auctionID + " from " + sourceID + " to " + destinationID);
			}
			
			//send estimation
			int estimation = calculateEstimation(sourceID, destinationID);
			ACLMessage msgOut = new ACLMessage(MessageType.SEND_ESTIMATION);
			
			msgOut.addUserDefinedParameter("auctionID", "" + auctionID);
			msgOut.addUserDefinedParameter("estimation", "" + estimation);
			
			AgentHelper.addReceivers(msgOut, myAgent, ((VehicleRoutingAgent)myAgent).getSzenarioID());
			
			if(Debugging.showAuctionMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " sent SEND_ESTIMATION message");
			
			send(msgOut);
			
		}

		public boolean done() {
			return false;
		}
	}
	
	private int calculateEstimation(int sourceID, int destinationID) {
		return -1;
	}
	
	private class InitializePacketAgentBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}
	
	private class IsFreeForTransportBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}
}
