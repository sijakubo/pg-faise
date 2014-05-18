package uni.oldenburg.server.agent;

import java.io.IOException;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

@SuppressWarnings("serial")
public class VehicleRoutingAgent extends Agent {
	public final static String NAME = "VehicleRoutingAgent";
	
	private int conveyorID = 0;
	private int szenarioID = 0;
	private boolean reserved=false;
	
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
		
		addBehaviour(new SendEstimationBehaviour(MessageTemplate.MatchPerformative(MessageType.START_AUCTION)));
		addBehaviour(new AssignVehicleForPackageBehaviour(MessageTemplate.MatchPerformative(MessageType.ASSIGN_VEHICLE_FOR_PACKAGE)));
		addBehaviour(new InitializePacketAgentBehaviour());
		addBehaviour(new IsFreeForTransportBehaviour());
		addBehaviour(new SetBotUnreservedBehaviour(MessageTemplate.MatchPerformative(MessageType.SET_BOT_UNRESERVED)));
		
		
		String nickname = AgentHelper.getUniqueNickname(VehicleRoutingAgent.NAME, conveyorID, szenarioID);		
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
	 * @author Christopher, sijakubo
	 */
	private class SendEstimationBehaviour extends CyclicReceiverBehaviour {

      int auctionID;
      int sourceID;
      int destinationID;

      protected SendEstimationBehaviour(MessageTemplate mt) {
         super(mt);
      }

      @Override
      public void onMessage(ACLMessage msgIn) throws UnreadableException, IOException {
         if (!reserved) {
            logger.log(Level.INFO, "VehicleRoutingAgent -> calculating estimation");

            auctionID = Integer.valueOf(msgIn.getUserDefinedParameter("auctionID"));
            sourceID = Integer.valueOf(msgIn.getUserDefinedParameter("sourceID"));
            destinationID = Integer.valueOf(msgIn.getUserDefinedParameter("destinationID"));

            logger.log(Level.INFO, myAgent.getLocalName() + " received START_AUCTION message #"
                  + auctionID + " from " + sourceID + " to " + destinationID);

            //send estimation
            int estimation = calculateEstimation(sourceID, destinationID);
            ACLMessage msgOut = new ACLMessage(MessageType.SEND_ESTIMATION);

            msgOut.addUserDefinedParameter("auctionID", "" + auctionID);
            msgOut.addUserDefinedParameter("vehicleID", "" + conveyorID);
            msgOut.addUserDefinedParameter("estimation", "" + estimation);
            msgOut.addUserDefinedParameter("destinationID", "" + destinationID);

            AgentHelper.addReceivers(msgOut, myAgent, ((VehicleRoutingAgent) myAgent).getSzenarioID());

            logger.log(Level.INFO, myAgent.getLocalName() + " sent SEND_ESTIMATION message with vehicleID "
                  + conveyorID + " auctionID " + auctionID + " and estimation: " + estimation);
            send(msgOut);
         } else {
            logger.log(Level.INFO, "VehicleRoutingAgent -> no estimation, Vehicle is already reserved");
         }
      }
	}
	
	private int calculateEstimation(int sourceID, int destinationID) {
		//pseudorandom values
		//TODO: pathfinding etc.
		return conveyorID + sourceID - destinationID;
	}

	public boolean isReserved() {
		return reserved;
	}

	public void setReserved(boolean reserved) {
		this.reserved = reserved;
	}

	/**
	 * @author Christopher, Raschid, sijakubo
	 */
	private class AssignVehicleForPackageBehaviour extends CyclicReceiverBehaviour {

      protected AssignVehicleForPackageBehaviour(MessageTemplate mt) {
         super(mt);
      }

      @Override
      public void onMessage(ACLMessage msgIn) throws UnreadableException, IOException {
         logger.log(Level.INFO, "VehicleRoutingAgent <- ASSIGN_VEHICLE_FOR_PACKAGE");

         setReserved(true);
         VehicleRoutingAgent currentAgent = (VehicleRoutingAgent) myAgent;
         int sourceID;
         int destinationID;
         int botID;
         int packageID;

         sourceID = Integer.valueOf(msgIn.getUserDefinedParameter("sourceID"));
         destinationID = Integer.valueOf(msgIn.getUserDefinedParameter("destinationID"));
         botID = Integer.valueOf(msgIn.getUserDefinedParameter("botID"));
         packageID = Integer.valueOf(msgIn.getUserDefinedParameter("packageID"));

         if (Debugging.showAuctionMessages) {
            logger.log(Level.INFO, myAgent.getLocalName() + " received ASSIGN_VEHICLE_FOR_TRANSPORT message for bot "
                  + botID + " to carry " + packageID + " from " + sourceID + " to " + destinationID);
         }

         //Tell the plattform Agent to get the Package from its Source
         ACLMessage msgStartGetting = new ACLMessage(MessageType.GET_PACKAGE_FROM_SOURCE);
         msgStartGetting.addUserDefinedParameter("destinationID", "" + destinationID);
         msgStartGetting.addUserDefinedParameter("sourceID", "" + sourceID);
         msgStartGetting.addUserDefinedParameter("packageID", "" + packageID);
         AgentHelper.addReceiver(msgStartGetting, myAgent, VehiclePlattformAgent.NAME, currentAgent.conveyorID, currentAgent.szenarioID);

      }
   }
	
	/**Got message:
	 *      VehiclePlattformAgent: BotGoToDestinationBehaviour           
	 * Send message:
	 *      none
	 * Behaviour should set the Bot unreserved
	 * @author Raschid
	 */
	private class SetBotUnreservedBehaviour extends
			CyclicReceiverBehaviour {

		protected SetBotUnreservedBehaviour (MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			//Receive the Message from Plattformagent and set the Status of the Bot unreserved
	
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- SET_BOT_UNRESERVED");
			setReserved(false);		
		}

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
