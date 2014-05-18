package uni.oldenburg.server.agent;

import java.awt.Dimension;
import java.io.IOException;
import java.util.Vector;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

@SuppressWarnings("serial")
public class RampRoutingAgent extends Agent {
	public final static String NAME = "RampRoutingAgent";

	private int conveyorID = 0;
	private int szenarioID = 0;

	private static int auctionIdCounter = 0;
	private int currentAuction = -1;
	private Vector<Dimension> estimations = new Vector<Dimension>();
	private long endOfAuction = -1;
	private int currentDestinationID = -1;

	private Logger logger = Logger.getLogger(RampRoutingAgent.class);

	/**
	 * @author Matthias Christopher
	 */
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			szenarioID = (Integer) args[0];

			Conveyor myConveyor = (Conveyor) args[1];
			conveyorID = myConveyor.getID();
		}
		
		addBehaviour(new StartAuctionBehaviour(MessageTemplate.MatchPerformative(MessageType.INITIALIZE_START_AUCTION_BEHAVIOUR)));
		addBehaviour(new ReceiveEstimationBehaviour());
		addBehaviour(new AssignVehicleForPackageBehaviour());

		String nickname = AgentHelper.getUniqueNickname(RampRoutingAgent.NAME,
				conveyorID, szenarioID);
		AgentHelper.registerAgent(szenarioID, this, nickname);

		if (Debugging.showAgentStartupMessages)
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
	private class StartAuctionBehaviour extends CyclicReceiverBehaviour {
		protected StartAuctionBehaviour(MessageTemplate mt) {
			super(mt);
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			// send message
			ACLMessage msgStart = new ACLMessage(MessageType.START_AUCTION);

			currentAuction = auctionIdCounter++;
			currentDestinationID = Integer.parseInt(msg.getUserDefinedParameter("conveyorId"));

			String auctionID = "" + currentAuction;
			String sourceID = "" + ((RampRoutingAgent) myAgent).getConveyorID();
			String destinationID = "" + currentDestinationID;

         msgStart.addUserDefinedParameter("auctionID", auctionID);
         msgStart.addUserDefinedParameter("sourceID", sourceID);
         msgStart.addUserDefinedParameter("destinationID", destinationID);

         AgentHelper.addReceivers(msgStart, myAgent, ((RampRoutingAgent) myAgent).getSzenarioID());

         logger.log(Level.INFO, myAgent.getLocalName() + " sent START_AUCTION message #" + auctionID
               + " from " + sourceID + " to " + destinationID);

         estimations.removeAllElements();
         endOfAuction = System.currentTimeMillis() + Debugging.auctionTimeout;
         send(msgStart);

		}
	}

	/**
	 * @author Christopher
	 */
	private class ReceiveEstimationBehaviour extends CyclicBehaviour {

		public void action() {

			int auctionID;
			int vehicleID;
			int estimation;

			// wait for message
			MessageTemplate mt = MessageTemplate
					.MatchPerformative(MessageType.SEND_ESTIMATION);
			ACLMessage msg = myAgent.receive(mt);

			if (msg != null) {
				try {
					auctionID = Integer.valueOf(msg
							.getUserDefinedParameter("auctionID"));
				} catch (NumberFormatException e) {
					auctionID = -1;
				}
				try {
					vehicleID = Integer.valueOf(msg
							.getUserDefinedParameter("vehicleID"));
				} catch (NumberFormatException e) {
					vehicleID = -1;
				}
				try {
					estimation = Integer.valueOf(msg
							.getUserDefinedParameter("estimation"));
				} catch (NumberFormatException e) {
					estimation = -1;
				}

				if (Debugging.showAuctionMessages) {
					logger.log(Level.INFO, myAgent.getLocalName()
							+ " received ESTIMATION message with vehicleID "
							+ vehicleID + " auctionID " + auctionID
							+ " and estimation: " + estimation);
				}

				if (auctionID == currentAuction) {
					estimations.add(new Dimension(vehicleID, estimation));
				}
			} else {
				block();
			}
		}
	}

	/**
	 * @author Christopher
	 */
	private class AssignVehicleForPackageBehaviour extends CyclicBehaviour {
		public void action() {

			if (endOfAuction != -1 && endOfAuction < System.currentTimeMillis()) {

				if (estimations.size() > 0) {
					Dimension bestEstimation = estimations.elementAt(0);

					for (Dimension estimation : estimations) {
						if (estimation.getHeight() < bestEstimation.getHeight()) { // height
																					// =
																					// estimation,
																					// width
																					// =
																					// botID
							bestEstimation = estimation;
						}
					}

					// send message
					ACLMessage msg = new ACLMessage(
							MessageType.ASSIGN_VEHICLE_FOR_PACKAGE);

					String sourceID = ""
							+ ((RampRoutingAgent) myAgent).getConveyorID();
					String destinationID = "" + currentDestinationID;
					String botID = "" + (int) bestEstimation.getWidth();
					String packageID = "tbd";

					msg.addUserDefinedParameter("sourceID", sourceID);
					msg.addUserDefinedParameter("destinationID", destinationID);
					msg.addUserDefinedParameter("botID", botID);
					msg.addUserDefinedParameter("packageID", packageID);

					AgentHelper.addReceivers(msg, myAgent,
							((RampRoutingAgent) myAgent).getSzenarioID());

					if (Debugging.showAuctionMessages) {
						logger.log(
								Level.INFO,
								myAgent.getLocalName()
										+ " sent ASSIGN_VEHICLE_FOR_TRANSPORT message for bot "
										+ botID + " to carry " + packageID
										+ " from " + sourceID + " to "
										+ destinationID);
					}

					send(msg);
				}

				endOfAuction = -1;
			} else {
				if (endOfAuction == -1) {
					block();
				} else {
					block(endOfAuction - System.currentTimeMillis());
				}
			}
		}
	}
}
