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
		
		if(Debugging.showAgentStartupMessages)logger.log(Level.INFO, nickname + " started");
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
	
	private class AssignVehicleForPackageBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}

	/**
	 * @author Christopher
	 */
	private class StartAuctionBehaviour extends OneShotBehaviour {
		public void action() {
			// send message
			ACLMessage msg = new ACLMessage(MessageType.START_AUCTION);
			
			String auctionId = "tbd";
			String sourceId = "" + ((RampRoutingAgent)myAgent).getConveyorID();
			String destinationId = "tbd";
			
			msg.addUserDefinedParameter("auctionId", auctionId);
			msg.addUserDefinedParameter("sourceId", sourceId);
			msg.addUserDefinedParameter("destinationId", destinationId);
			
			AgentHelper.addReceivers(((RampRoutingAgent)myAgent).getSzenarioID(), myAgent, msg);
			if(Debugging.showAuctionMessages)logger.log(Level.INFO, myAgent.getLocalName() + " sent START_AUCTION message");
			send(msg);
		}
	}
	
	private class ReceiveEstimationBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}
}
