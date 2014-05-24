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
import uni.oldenburg.shared.model.ConveyorRamp;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import uni.oldenburg.shared.model.Szenario;

@SuppressWarnings("serial")
public class RampRoutingAgent extends Agent {
	public final static String NAME = "RampRoutingAgent";

	private int conveyorID = 0;
	private Szenario szenario;
	private int actualPackageId = -1;
	private static int auctionIdCounter = 0;
	private int currentAuction = -1;
	private Vector<Dimension> estimations = new Vector<Dimension>();
	private long endOfAuction = -1;
	private int currentDestinationID = -1;
	private boolean auctionStarted = false;
	private int rampType = -1; //-1 represents a Vehicle
	private Logger logger = Logger.getLogger(RampRoutingAgent.class);

	/**
	 * @author Matthias Christopher
	 */
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			szenario = (Szenario) args[0];

			Conveyor myConveyor = (Conveyor) args[1];
			conveyorID = myConveyor.getID();
			if (myConveyor instanceof ConveyorRamp) {
	            rampType = ((ConveyorRamp) myConveyor).getRampType();
	         }
		}

		if(rampType==ConveyorRamp.RAMP_ENTRANCE|| rampType == ConveyorRamp.RAMP_STOREAGE){
			addBehaviour(new StartAuctionBehaviour(
					MessageTemplate
							.MatchPerformative(MessageType.INITIALIZE_START_AUCTION_BEHAVIOUR)));
			addBehaviour(new ReceiveEstimationBehaviour(
					MessageTemplate.MatchPerformative(MessageType.SEND_ESTIMATION)));
			addBehaviour(new AssignVehicleForPackageBehaviour());
			
		}
		

		String nickname = AgentHelper.getUniqueNickname(RampRoutingAgent.NAME, conveyorID, szenario.getId());
		AgentHelper.registerAgent(szenario.getId(), this, nickname);

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

	public int getActualPackageId() {
		return actualPackageId;
	}

	public void setActualPackageId(int actualPackageId) {
		this.actualPackageId = actualPackageId;
	}

	public boolean isAuctionStarted() {
		return auctionStarted;
	}

	public void setAuctionStarted(boolean auctionStarted) {
		this.auctionStarted = auctionStarted;
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
			// Check if an Auction was already started
			if (auctionStarted == false) {
				setAuctionStarted(true);
				// Set the actual Package id
				setActualPackageId(Integer.parseInt(msg
						.getUserDefinedParameter("packageId")));
				// send message
				ACLMessage msgStart = new ACLMessage(MessageType.START_AUCTION);

				currentAuction = auctionIdCounter++;
				currentDestinationID = Integer.parseInt(msg
						.getUserDefinedParameter("conveyorId"));

				String auctionID = "" + currentAuction;
				String sourceID = ""
						+ ((RampRoutingAgent) myAgent).getConveyorID();
				String destinationID = "" + currentDestinationID;

				msgStart.addUserDefinedParameter("auctionID", auctionID);
				msgStart.addUserDefinedParameter("sourceID", sourceID);
				msgStart.addUserDefinedParameter("destinationID", destinationID);

				AgentHelper.addReceivers(msgStart, myAgent, szenario.getId());

				logger.log(Level.INFO, myAgent.getLocalName()
						+ " sent START_AUCTION message #" + auctionID
						+ " from " + sourceID + " to " + destinationID);

				estimations.removeAllElements();
				endOfAuction = System.currentTimeMillis()
						+ Debugging.auctionTimeout;
				send(msgStart);
			}
		}
	}

	/**
	 * @author Christopher, sijakubo
	 */
	private class ReceiveEstimationBehaviour extends CyclicReceiverBehaviour {

		protected ReceiveEstimationBehaviour(MessageTemplate mt) {
			super(mt);
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			int auctionID = Integer.valueOf(msg
					.getUserDefinedParameter("auctionID"));
			int vehicleID = Integer.valueOf(msg
					.getUserDefinedParameter("vehicleID"));
			int estimation = Integer.valueOf(msg
					.getUserDefinedParameter("estimation"));

			if (auctionID == currentAuction) {
				logger.log(Level.INFO, myAgent.getLocalName()
						+ " received ESTIMATION message with vehicleID "
						+ vehicleID + " auctionID " + auctionID
						+ " and estimation: " + estimation);

				estimations.add(new Dimension(vehicleID, estimation));
			}
		}
	}

	/**
	 * @author Christopher
	 */
	private class AssignVehicleForPackageBehaviour extends CyclicBehaviour {
		public void action() {
			RampRoutingAgent currentAgent = (RampRoutingAgent) myAgent;
			if (endOfAuction != -1 && endOfAuction < System.currentTimeMillis()) {
				logger.log(Level.INFO, "Auction will be finished");

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
					String packageID = "" + actualPackageId;

					msg.addUserDefinedParameter("sourceID", sourceID);
					msg.addUserDefinedParameter("destinationID", destinationID);
					msg.addUserDefinedParameter("botID", botID);
					msg.addUserDefinedParameter("packageID", packageID);

					// AgentHelper.addReceivers(msg, myAgent,
					// ((RampRoutingAgent) myAgent).getszenario.getId()());
					AgentHelper.addReceiver(msg, currentAgent,
							VehicleRoutingAgent.NAME, Integer.parseInt(botID),
							szenario.getId());
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
				setAuctionStarted(false);
				logger.log(Level.INFO, "Auction is finished");
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
