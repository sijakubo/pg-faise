package uni.oldenburg.server.agent;

import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.client.view.MainFrameView;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.server.pathfinding.GridItem;
import uni.oldenburg.server.pathfinding.IPathfinding;
import uni.oldenburg.server.pathfinding.PathPoint;
import uni.oldenburg.server.pathfinding.Pathfinding;
import uni.oldenburg.server.pathfinding.GridItem.GridItemType;
import uni.oldenburg.server.pathfinding.Pathfinding.PathMessageType;
import uni.oldenburg.server.pathfinding.PathfindingSingle;
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
	//Interface für das Pathfinding
	private IPathfinding myPF = null;
	
	private Logger logger = Logger.getLogger(VehicleRoutingAgent.class);
    //Speichert die Punkte für das Pathfinding
	private List<List<PathPoint>> lstPathPoints = new ArrayList<List<PathPoint>>(); 

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
		
		int myColumnCount = MainFrameView.canvasWidth / Conveyor.RASTER_SIZE;
		int myRowCount = MainFrameView.canvasHeight / Conveyor.RASTER_SIZE;
		//Erzeugt für jede Kachel ein Griditem und speichert es in der Liste 
		List<GridItem> lstGridItem = new ArrayList<GridItem>();
		
		for (int i = 0; i < myColumnCount * myRowCount; ++i) {
			lstGridItem.add(new GridItem(1));
		}
		
		logger.log(Level.INFO, "w/h: " + myColumnCount + " / " + myRowCount);
		//Alle Conveyor durchlaufen
		for(Conveyor myConveyor : mySzenario.getConveyorList()) {
			if (!(myConveyor instanceof ConveyorVehicle)) { //Es wird geprüft, ob Conveyor Fahrzeug ist.
				int x = myConveyor.getX();
				int y = myConveyor.getY();
				
				logger.log(Level.INFO, "x/y: " + x + " / " + y + " -> " + Pathfinding.getIndex(x, y, myColumnCount));
				//Jeder Conveyor, der kein Fahrzeug ist, wird im entsprechenden Griditem als Wand deklariert und somit als Hinderniss gekennzeichnet.
				GridItem myItem = lstGridItem.get(Pathfinding.getIndex(x, y, myColumnCount));
				myItem.setItemType(GridItemType.WallItem);
			}
		}
		
		Pathfinding.drawGrid(myColumnCount, myRowCount, lstGridItem);
		
		//Initialisierung mit PathfindingSingle
		myPF = new PathfindingSingle(myColumnCount, myRowCount, lstGridItem);
		
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
	 * Bekommt Nachricht von:
	 *		RampRoutingAgent::Auction
	 *		VehiclePlattformAgent::GetCurrentPosition		 		
	 * Sendet Nachricht an:
	 * 		VehiclePlattformAgent::GetCurrentPosition
	 * 		RampRoutingAgent::Auction
	 * 
	 * Verarbeitet Estimation Anfragen.
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
			
			if (hasPendingJob)
				logger.log(Level.INFO, "Vehicle: " + myConveyor.getID() + " - has pending job");
			
			if (auctionInProgress == false && hasPendingJob == false) {
				auctionInProgress = true;
				
				// request current position
				ACLMessage msgPositionRequest = new ACLMessage(MessageType.GET_CURRENT_POSITION);
				AgentHelper.addReceiver(msgPositionRequest, myAgent, VehiclePlattformAgent.NAME, myConveyor.getID(), mySzenario.getId());
				send(msgPositionRequest);
				
				// get current position
				ACLMessage msgPositionResponse = myAgent.blockingReceive(MessageTemplate.MatchPerformative(MessageType.GET_CURRENT_POSITION));
				int cur_x = Integer.parseInt(msgPositionResponse.getUserDefinedParameter("pos_x"));
				int cur_y = Integer.parseInt(msgPositionResponse.getUserDefinedParameter("pos_y"));
				Point curPoint = new Point(cur_x, cur_y);
				
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
				// calculate estimations
				if (lstPathPoints != null) {
					for (List<PathPoint> lstPoints : lstPathPoints) {
						if (lstPoints != null)
							lstPoints.clear();
					}
					
					lstPathPoints.clear();
						
				}
				
				int toSourceRampEstimation = CalculateEstimation(curPoint, srcRampPoint);
				//int toDestinationRampEstimation = CalculateEstimation(srcRampPoint, dstRampPoint);
				
				sumEstimation = toSourceRampEstimation; //+ toDestinationRampEstimation;
				

				if (toSourceRampEstimation < 0);// || toDestinationRampEstimation < 0)
					sumEstimation = -1;
			}
			else {
				hasPendingJob = true;
			}
			
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
     * @author Matthias, Nagi
     */
	private class AssignJob extends CyclicReceiverBehaviour {
		protected AssignJob(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			int vehicleWhoGotJobID = Integer.parseInt(msg.getUserDefinedParameter("vehicleID"));
			
			// am i the one who got the job?
			if (myConveyor.getID() == vehicleWhoGotJobID) {	
				// set pending incoming job status in own package-agent
				ACLMessage msgSetPendingStatus = new ACLMessage(MessageType.SET_PENDING_INCOMING_STATUS);
				AgentHelper.addReceiver(msgSetPendingStatus, myAgent, PackageAgent.NAME, myConveyor.getID(), mySzenario.getId());
				send(msgSetPendingStatus);
				
				// send me "VehiclePlattformAgent" a message where to drive to
				ACLMessage msgSendPaths = new ACLMessage(MessageType.DRIVING_START);
				msgSendPaths.addUserDefinedParameter("srcRampID", "" + srcRampID);
				msgSendPaths.addUserDefinedParameter("dstRampID", "" + dstRampID);
				msgSendPaths.setContentObject((Serializable) lstPathPoints);
				
				AgentHelper.addReceiver(msgSendPaths, myAgent, VehiclePlattformAgent.NAME, myConveyor.getID(), mySzenario.getId());
				send(msgSendPaths);
			}
			
			auctionInProgress = false;
		}
	}
	
	@SuppressWarnings("null")
	/**
	 * Berechnet mithilfe des Pathfindingalgorithmus einen Pfad und die daraus resultierende Estimation
	 * 
     * @author Matthias
     */
	private int CalculateEstimation(Point startPoint, Point stopPoint) {
		List<List<PathPoint>> lstPathPointsTmp = null;//Liste mit Listen von Pathfinding Punkten. Berücksichtigt, dass ein theorethisch mehrere Weg berechnen kann.
				
		lstPathPointsTmp = myPF.findPath(startPoint, stopPoint);

		if (lstPathPointsTmp == null)
			System.out.println("2");
		
		if (lstPathPointsTmp == null)
			if (myPF.getError() != PathMessageType.PathFound)
				return -1;
			
		lstPathPoints.add(lstPathPointsTmp.get(0));		
	
		
		int sumEstimation = 0;//Variable speichert die Summe der Estimations-> 
		
		List<PathPoint> lstPoints = lstPathPointsTmp.get(0);//Liste von Punkten wird geliefert, die der Pathfinding Algorithmus gefunden hat.
		
		for(PathPoint tmpPathPoint : lstPoints) {
			sumEstimation += tmpPathPoint.getEstimationValue();//Estimations für die einzelnen Kacheln werden zusammengezählt.
		}
		
		logger.log(Level.INFO, "Est: " + sumEstimation + " : " + myPF.getError().toString());
		
		return sumEstimation;
	}
}
