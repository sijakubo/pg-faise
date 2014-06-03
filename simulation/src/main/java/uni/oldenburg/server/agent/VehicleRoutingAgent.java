package uni.oldenburg.server.agent;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.io.IOException;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.ConveyorVehicle;
import uni.oldenburg.shared.model.Point;
import uni.oldenburg.shared.model.Szenario;

@SuppressWarnings("serial")
public class VehicleRoutingAgent extends Agent {
	public final static String NAME = "VehicleRoutingAgent";

	private ConveyorVehicle myConveyor;
	private Szenario mySzenario;
	
	private int srcRampID = 0;
	private int dstRampID = 0;
	
	private Point srcRampPoint = null;
	private Point dstRampPoint = null;
	
	private boolean auctionInProgress = false;
	
	private Logger logger = Logger.getLogger(VehicleRoutingAgent.class);

	/**
     * @author Matthias
     */
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			mySzenario = (Szenario) args[0];
			myConveyor = (ConveyorVehicle) args[1];
		}
		
		addBehaviour(new EstimationRequest(MessageType.ESTIMATION_REQUEST));
		addBehaviour(new AssignJob(MessageType.ASSIGN_JOB_TO_VEHICLE));

		String nickname = AgentHelper.getUniqueNickname(VehicleRoutingAgent.NAME, myConveyor.getID(), mySzenario.getId());
		AgentHelper.registerAgent(mySzenario.getId(), this, nickname);

		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}

	// destructor
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * Got message:
	 *		RampRoutingAgent::Auction
	 *		VehiclePlattformAgent::GetCurrentPosition		 		
	 * Send message:
	 * 		VehiclePlattformAgent::GetCurrentPosition
	 * 		RampRoutingAgent::Auction
	 * 
	 * handles estimation requests
	 * 
     * @author Matthias
     */

	private class EstimationRequest extends CyclicReceiverBehaviour {
		protected EstimationRequest(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			int sumEstimation = 1;
			boolean hasPendingJob = false;
			
			ACLMessage msgGetPendingJobStatus = new ACLMessage(MessageType.GET_PENDING_JOB_STATUS);
			AgentHelper.addReceiver(msgGetPendingJobStatus, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
			send(msgGetPendingJobStatus);
			
			ACLMessage msgPendingJobStatus = myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.GET_PENDING_JOB_STATUS));
			hasPendingJob = msgPendingJobStatus.getUserDefinedParameter("pendingJob").equals("1") ? true : false;
			
			//logger.log(Level.INFO, "Request estimation in process...");
			
			//logger.log(Level.INFO, "Conveyor " + myConveyor.getID() + " has job: " + hasPendingJob);
			
			if (auctionInProgress == false && hasPendingJob == false) {
				auctionInProgress = true;
				
				//logger.log(Level.INFO, "No auction in progress -> request current position");
				
				// request current position
				ACLMessage msgPositionRequest = new ACLMessage(MessageType.GET_CURRENT_POSITION);
				AgentHelper.addReceiver(msgPositionRequest, myAgent, VehiclePlattformAgent.NAME, myConveyor.getID(), mySzenario.getId());
				send(msgPositionRequest);
				
				// get current position
				ACLMessage msgPositionResponse = myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.GET_CURRENT_POSITION));
				int cur_x = Integer.parseInt(msgPositionResponse.getUserDefinedParameter("pos_x"));
				int cur_y = Integer.parseInt(msgPositionResponse.getUserDefinedParameter("pos_y"));
				Point curPoint = new Point(cur_x, cur_y);
				
				//logger.log(Level.INFO, "Got current position: " + curPoint.toString());
				
				srcRampID = Integer.parseInt(msg.getUserDefinedParameter("srcRampID"));
				dstRampID = Integer.parseInt(msg.getUserDefinedParameter("dstRampID"));
				
				// get entry/exit positions of ramps
				for(Conveyor tmpConveyor: mySzenario.getConveyorList()) {
					if (tmpConveyor instanceof ConveyorRamp) {
						ConveyorRamp tmpRampConveyor = (ConveyorRamp)tmpConveyor;
						
						if (tmpRampConveyor.getID() == srcRampID) {
							srcRampPoint = tmpRampConveyor.getExitPosition();
						}
						else if (tmpRampConveyor.getID() == dstRampID) {
							dstRampPoint = tmpRampConveyor.getEntryPosition();
						}
					}
				}
				
				//logger.log(Level.INFO, "SRC position: " + srcRampPoint.toString());
				//logger.log(Level.INFO, "DST position: " + dstRampPoint.toString());
				
				// calculate estimations
				int toSourceRampEstimation = CalculateEstimation(curPoint, srcRampPoint);
				int toDestinationRampEstimation = CalculateEstimation(srcRampPoint, dstRampPoint);
				
				sumEstimation = toSourceRampEstimation + toDestinationRampEstimation;
				
				//logger.log(Level.INFO, "Estimation: " + sumEstimation);
			}
			else {
				hasPendingJob = true;
			}
			
			if (hasPendingJob)
				logger.log(Level.INFO, "INFO: Conveyor " + myConveyor.getID() + " already has a pending job");
			
			// send estimation response
			ACLMessage msgEstimationResponse = new ACLMessage(MessageType.ESTIMATION_RESPONSE);
			msgEstimationResponse.addUserDefinedParameter("estimation", "" + sumEstimation);
			msgEstimationResponse.addUserDefinedParameter("vehicleID", "" + myConveyor.getID());
			
			msgEstimationResponse.addUserDefinedParameter("pendingJob", hasPendingJob ? "1" : "0");
			msgEstimationResponse.addReceiver(msg.getSender());
			send(msgEstimationResponse);			
		}
	}
		
	/**
	 * Got message:
	 *		RampRoutingAgent::Auction		 		
	 * Send message:
	 * 		PackageAgent::SetPendingIncomingStatus
	 * 		VehiclePlattformAgent::DrivePath
	 * 
	 * assign job to current vehicle when ids match
	 * 
     * @author Matthias
     */
	private class AssignJob extends CyclicReceiverBehaviour {
		protected AssignJob(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			int vehicleWhoGotJobID = Integer.parseInt(msg.getUserDefinedParameter("vehicleID"));
			
			//logger.log(Level.INFO, "Assign Job...");
			
			// am i the one who got the job?
			if (myConveyor.getID() == vehicleWhoGotJobID) {
				logger.log(Level.INFO, "Set Conveyor " + myConveyor.getID() + " incoming flag status");	
				
				// set pending incoming job status in own package-agent
				ACLMessage msgSetPendingStatus = new ACLMessage(MessageType.SET_PENDING_INCOMING_STATUS);
				AgentHelper.addReceiver(msgSetPendingStatus, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
				send(msgSetPendingStatus);
				
				//logger.log(Level.INFO, "Conveyor " + myConveyor.getID() + " -> start driving!");
				
				// send me "VehiclePlattformAgent" a message where to drive to
				ACLMessage msgSendPaths = new ACLMessage(MessageType.DRIVING_START);
				msgSendPaths.addUserDefinedParameter("srcRampID", "" + srcRampID);
				msgSendPaths.addUserDefinedParameter("dstRampID", "" + dstRampID);
				
				AgentHelper.addReceiver(msgSendPaths, myAgent, VehiclePlattformAgent.NAME, myConveyor.getID(), mySzenario.getId());
				send(msgSendPaths);
			}
			
			auctionInProgress = false;
		}
	}
	
	private int CalculateEstimation(Point startPoint, Point stopPoint) {
		return (int)(Math.random() * 100);
	}
}
