package uni.oldenburg.server.agent;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import uni.oldenburg.shared.model.Szenario;

/// starte auktion (hole estimations) -> vehiclerouting
// bekomme estimation <- vehiclerouting
// wähle fahrzeug für paket aus -> vehiclerouting

@SuppressWarnings("serial")
public class RampRoutingAgent extends Agent {
	public final static String NAME = "RampRoutingAgent";

	private Conveyor myConveyor;
	private Szenario mySzenario;
	private Logger logger = Logger.getLogger(RampRoutingAgent.class);
	
	private int vehicleCount = 0;

	/**
	 * @author Matthias
	 */
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			mySzenario = (Szenario) args[0];
			myConveyor = (Conveyor) args[1];
		}		
		
		addBehaviour(new Auction());
		addBehaviour(new SetPendingIncomingStatusRelay(MessageType.SET_PENDING_INCOMING_STATUS));
		
		vehicleCount = AgentHelper.getSimulationConveyorCounts(mySzenario).VehicleCount;

		String nickname = AgentHelper.getUniqueNickname(RampRoutingAgent.NAME, myConveyor.getID(), mySzenario.getId());
		AgentHelper.registerAgent(mySzenario.getId(), this, nickname);

		if (Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}

	// destructor
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * Got message:
	 *		RampOrderAgent::SendEquirePackageRequestRelay
	 *		VehicleRoutingAgent::HandleEstimationRequestAssignment		 		
	 * Send message:
	 * 		VehicleRoutingAgent::HandleEstimationRequestAssignment
	 * 		RampRoutingAgent::SetPendingIncomingStatusRelay
	 * 		RampOrderAgent::SendEquirePackageRequestRelay
	 * 
	 * handles complete auction process
	 * 
     * @author Matthias
     */
	private class Auction extends CyclicBehaviour {
		int step = 0;
		int vehiclesAnswered = 0;
		Map<Integer, Integer> mapVehicleEstimation = new HashMap<Integer, Integer>();
		
		int srcRampID = 0;
		int dstRampID = 0;
		
		public void action() {
			// received auction start trigger -> request estimation requests
			if (step == 0) {
				ACLMessage msgReceive = myAgent.receive(MessageTemplate.MatchPerformative(MessageType.AUCTION_START));
				
				if (msgReceive != null) {
					srcRampID = Integer.parseInt(msgReceive.getUserDefinedParameter("srcRampID"));
					dstRampID = Integer.parseInt(msgReceive.getUserDefinedParameter("dstRampID"));
					
					// request estimation
					ACLMessage msgEstimationRequest = new ACLMessage(MessageType.ESTIMATION_REQUEST);
					msgEstimationRequest.addUserDefinedParameter("srcRampID", "" + srcRampID);
					msgEstimationRequest.addUserDefinedParameter("dstRampID", "" + dstRampID);
					AgentHelper.addReceivers(msgEstimationRequest, myAgent, mySzenario.getId());
					send(msgEstimationRequest);
					
					step = 1;
				}
				else
					block();
			}
			// collect estimations from all vehicles
			else if (step == 1) {
				ACLMessage msgReceive = myAgent.receive(MessageTemplate.MatchPerformative(MessageType.ESTIMATION_RESPONSE));
				
				if (msgReceive != null) {
					if (vehiclesAnswered < vehicleCount) {
						++vehiclesAnswered;
						
						// remember vehicles, who don't have a pending job
						if (msgReceive.getUserDefinedParameter("pendingJob").equals("0")) {	
							mapVehicleEstimation.put(
									Integer.parseInt(msgReceive.getUserDefinedParameter("vehicleID")),
									Integer.parseInt(msgReceive.getUserDefinedParameter("estimation")));
						}
					}
					else {
						step = 2;
						vehiclesAnswered = 0;
					}
				}
				else
					block();
			}
			// choose vehicle with best estimation
			else if (step == 2) {
				int bestVehicleID = -1;
				int bestVehicleEstimation = 0;
				List<Integer> lstBestVehicles = new ArrayList<Integer>();
				
				// find best estimation value from all vehicles who answered without a pending job
				for(Integer myVehicleEstimation : mapVehicleEstimation.values()) {
					if (bestVehicleEstimation <= 0 || myVehicleEstimation < bestVehicleEstimation) {
						bestVehicleEstimation = myVehicleEstimation;
					}
				}
				
				// collect vehicles with the best estimation (in case there is more than one)
				for(Integer myVehicleID : mapVehicleEstimation.keySet()) {
					if (mapVehicleEstimation.get(myVehicleID) <= bestVehicleEstimation) {
						lstBestVehicles.add(myVehicleID);
					}
				}
				
				// choose a random vehicle with the best estimation
				if (lstBestVehicles.size() > 0) {					
					int randomIndex = (int)(Math.random() * 100) % lstBestVehicles.size();
					
					bestVehicleID = lstBestVehicles.get(randomIndex);
					
					// set pending incoming job status in destination ramp
					ACLMessage msgSetPendingStatus = new ACLMessage(MessageType.SET_PENDING_INCOMING_STATUS);
					AgentHelper.addReceiver(msgSetPendingStatus, myAgent, RampRoutingAgent.NAME, dstRampID, mySzenario.getId());
					send(msgSetPendingStatus);
					
					// send message to all "VehicleRoutingAgents" about who got the job
					ACLMessage msgAssignJob = new ACLMessage(MessageType.ASSIGN_JOB_TO_VEHICLE);
					msgAssignJob.addUserDefinedParameter("vehicleID", "" + bestVehicleID);
					msgAssignJob.addUserDefinedParameter("srcRampID", "" + srcRampID);
					msgAssignJob.addUserDefinedParameter("dstRampID", "" + dstRampID);
					AgentHelper.addReceivers(msgAssignJob, myAgent, mySzenario.getId());
					send(msgAssignJob);
					
					// auction has ended and "hopefully" a vehicle has been found
					ACLMessage msgAuctionEnd = new ACLMessage(MessageType.AUCTION_END);
					msgAuctionEnd.addUserDefinedParameter("vehicle_found", (bestVehicleID > -1) ? "1" : "0");
					msgAuctionEnd.addUserDefinedParameter("vehicleID", "" + bestVehicleID);
					AgentHelper.addReceiver(msgAuctionEnd, myAgent, RampOrderAgent.NAME, myConveyor.getID(), mySzenario.getId());
					send(msgAuctionEnd);
				}
				
				step = 0;
			}
		}
	}
	
	/**
	 * Got message:
	 *		RampRoutingAgent::Auction	 		
	 * Send message:
	 * 		PackageAgent::SetPendingIncomingStatus
	 * 
	 * sets pending incoming job flag in own package-agent
	 * 
     * @author Matthias
     */
	private class SetPendingIncomingStatusRelay extends CyclicReceiverBehaviour {
		protected SetPendingIncomingStatusRelay(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			// set pending incoming job status to own package-agent
			ACLMessage msgSetPendingStatus = new ACLMessage(MessageType.SET_PENDING_INCOMING_STATUS);
			AgentHelper.addReceiver(msgSetPendingStatus, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
			send(msgSetPendingStatus);
		}		
	}
}
