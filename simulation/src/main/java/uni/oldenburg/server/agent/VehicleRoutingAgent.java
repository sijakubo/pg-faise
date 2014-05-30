package uni.oldenburg.server.agent;

import java.awt.Point;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import uni.oldenburg.shared.model.Szenario;

/// bekomme anfrage für estimation
/// berechne estimation
/// sende estimation

/// 1. bot wurde nicht ausgewählt
///		-> pech gehabt
/// 2. bot wurde ausgewählt
///		-> VehiclePlattform -> fahre zum eingang

@SuppressWarnings("serial")
public class VehicleRoutingAgent extends Agent {
	public final static String NAME = "VehicleRoutingAgent";

	private Conveyor myConveyor;
	private Szenario mySzenario;
	
	private Logger logger = Logger.getLogger(VehicleRoutingAgent.class);

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
		
		addBehaviour(new HandleEstimationRequestAssignment());

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
	 * Send message:
	 * 		RampRoutingAgent::Auction
	 * 		PackageAgent::SetPendingIncomingStatus
	 * 		VehiclePlattformAgent::DrivePath
	 * 
	 * handles complete auction process
	 * 
     * @author Matthias
     */
	private class HandleEstimationRequestAssignment extends CyclicBehaviour {
		int step = 0;
		Point srcRampPoint = null;
		Point dstRampPoint = null;
		
		public void action() {
			// send estimation upon request
			if (step == 0) {
				ACLMessage msgReceive = myAgent.receive(MessageTemplate.MatchPerformative(MessageType.ESTIMATION_REQUEST));
				
				if (msgReceive != null) {
					// request current position
					ACLMessage msgPositionRequest = new ACLMessage(MessageType.GET_CURRENT_POSITION);
					AgentHelper.addReceiver(msgPositionRequest, myAgent, VehiclePlattformAgent.NAME, myConveyor.getID(), mySzenario.getId());
					send(msgPositionRequest);
					
					// get current position
					ACLMessage msgPositionResponse = myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.GET_CURRENT_POSITION));
					int cur_x = Integer.parseInt(msgPositionResponse.getUserDefinedParameter("pos_x"));
					int cur_y = Integer.parseInt(msgPositionResponse.getUserDefinedParameter("pos_y"));
					Point curPoint = new Point(cur_x, cur_y);
					
					int srcRampID = Integer.parseInt(msgReceive.getUserDefinedParameter("srcRampID"));
					int dstRampID = Integer.parseInt(msgReceive.getUserDefinedParameter("dstRampID"));
					
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
					
					// calculate estimations
					int toSourceRampEstimation = CalculateEstimation(curPoint, srcRampPoint);
					int toDestinationRampEstimation = CalculateEstimation(srcRampPoint, dstRampPoint);
					
					int sumEstimation = toSourceRampEstimation + toDestinationRampEstimation;
					
					boolean hasPendingJob = false;
					
					// send estimation response
					ACLMessage msgEstimationResponse = new ACLMessage(MessageType.ESTIMATION_RESPONSE);
					msgEstimationResponse.addUserDefinedParameter("estimation", "" + sumEstimation);
					msgEstimationResponse.addUserDefinedParameter("vehicleID", "" + myConveyor.getID());
					msgEstimationResponse.addUserDefinedParameter("pendingJob", hasPendingJob ? "1" : "0");
					msgEstimationResponse.addReceiver(msgReceive.getSender());
					send(msgEstimationResponse);
					
					step = 1;
				}
				else
					block();
			}
			// wait for message if you got chosen or not
			else if (step == 1) {
				int vehicleWhoGotJobID = 0;
				
				ACLMessage msgAssignJob = myAgent.receive(MessageTemplate.MatchPerformative(MessageType.ASSIGN_JOB_TO_VEHICLE));
				
				if (msgAssignJob != null) {
					vehicleWhoGotJobID = Integer.parseInt(msgAssignJob.getUserDefinedParameter("vehicleID"));
					
					// am i the one who got the job?
					if (myConveyor.getID() == vehicleWhoGotJobID) {
						// set pending incoming job status in own package-agent
						ACLMessage msgSetPendingStatus = new ACLMessage(MessageType.SET_PENDING_INCOMING_STATUS);
						AgentHelper.addReceiver(msgSetPendingStatus, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
						send(msgSetPendingStatus);
						
						// send me "VehiclePlattformAgent" a message where to drive to
						ACLMessage msgSendPaths = new ACLMessage(MessageType.PATHS_SEND);
						//msgSendPaths.addUserDefinedParameter("srcRampID", "" + Integer.parseInt(msgAssignJob.getUserDefinedParameter("srcRampID")));
						//msgSendPaths.addUserDefinedParameter("dstRampID", "" + Integer.parseInt(msgAssignJob.getUserDefinedParameter("dstRampID")));
						
						msgSendPaths.addUserDefinedParameter("point_x_to_source", 		"" + this.srcRampPoint.x);
						msgSendPaths.addUserDefinedParameter("point_y_to_source", 		"" + this.srcRampPoint.y);
						msgSendPaths.addUserDefinedParameter("point_x_to_destionation", "" + this.dstRampPoint.x);
						msgSendPaths.addUserDefinedParameter("point_y_to_destionation", "" + this.dstRampPoint.y);
						
						AgentHelper.addReceiver(msgSendPaths, myAgent, VehiclePlattformAgent.NAME, myConveyor.getID(), mySzenario.getId());
						send(msgSendPaths);
					}
					
					step = 0;
				}
				else
					block();
			}
		}		
	}
	
	private int CalculateEstimation(Point startPoint, Point stopPoint) {
		return (int)(Math.random() * 100);
	}
}
