package uni.oldenburg.server.agent;

import java.awt.Point;
import java.io.IOException;

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

/// fahre zur position xy (eingang)
/// nehme paket von der rampe runter -> rampplatform
/// fahre zur position xy (ausgang)
/// gebe paket an die zielrampe -> rampplatform
/// fahre zur position xy (weg vom ausgang, damit platz frei wird)


@SuppressWarnings("serial")
public class VehiclePlattformAgent extends Agent {
	public final static String NAME = "VehiclePlattformAgent";
	
	private Conveyor myConveyor;
	private Szenario mySzenario;
	
	private Logger logger = Logger.getLogger(VehiclePlattformAgent.class);
	
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
		
		addBehaviour(new DrivePath());
		addBehaviour(new GetCurrentPosition(MessageType.GET_CURRENT_POSITION));
		
		String nickname = AgentHelper.getUniqueNickname(VehiclePlattformAgent.NAME, myConveyor.getID(), mySzenario.getId());		
		AgentHelper.registerAgent(mySzenario.getId(), this, nickname);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	private class GetCurrentPosition extends CyclicReceiverBehaviour {
		protected GetCurrentPosition(int msgType) {
			super(MessageTemplate.MatchPerformative(msgType));
		}

		public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
			ACLMessage msgResponse = new ACLMessage(MessageType.GET_CURRENT_POSITION);
			msgResponse.addReceiver(msg.getSender());
			
			msgResponse.addUserDefinedParameter("pos_x", "" + myConveyor.getX());
			msgResponse.addUserDefinedParameter("pos_y", "" + myConveyor.getY());
			
			send(msgResponse);
		}
	}
	
	private class DrivePath extends CyclicBehaviour {
		int step = 0;
		//Point curPoint = new Point(myConveyor.getX(), myConveyor.getY());
		Point srcPoint = null;
		Point dstPoint = null;
		
		public void action() {
			// start driving
			if (step == 0) {
				ACLMessage msgResponse = myAgent.receive(MessageTemplate.MatchPerformative(MessageType.PATHS_SEND));
				
				if (msgResponse != null) {
					int srcPosX = Integer.parseInt(msgResponse.getUserDefinedParameter("point_x_to_source"));
					int srcPosY = Integer.parseInt(msgResponse.getUserDefinedParameter("point_y_to_source"));
					srcPoint = new Point(srcPosX, srcPosY);
					
					int dstPosX = Integer.parseInt(msgResponse.getUserDefinedParameter("point_x_to_destionation"));
					int dstPosY = Integer.parseInt(msgResponse.getUserDefinedParameter("point_y_to_destionation"));
					dstPoint = new Point(dstPosX, dstPosY);
					
					step = 1;
				}
				else
					block();
			}
			else if (step == 1) {
				// go to source ramp
				myConveyor.setPosition(srcPoint.x, srcPoint.y);
				
				
				// take package from source ramp
				
				// go to destination ramp				
				myConveyor.setPosition(dstPoint.x, dstPoint.y);
				
				// give package to destination ramp				
				
				step = 0;
			}
		}
	}
}
