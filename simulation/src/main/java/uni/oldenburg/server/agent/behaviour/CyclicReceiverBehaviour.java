package uni.oldenburg.server.agent.behaviour;

import java.io.IOException;

import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

/**
 * @author Matthias
 */	
@SuppressWarnings("serial")
public abstract class CyclicReceiverBehaviour extends CyclicBehaviour {
	MessageTemplate mt = null;
	ACLMessage msg = null;
	
	public abstract void onMessage(ACLMessage msg) throws UnreadableException, IOException;
	
	protected CyclicReceiverBehaviour(MessageTemplate mt) {
		this.mt = mt;
	}
	
	public void action() {
		if (mt == null)
			msg = myAgent.receive();
		else
			msg = myAgent.receive(mt);
		
		if (msg != null) {
			try {
				onMessage(msg);
			} catch (UnreadableException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		else {
			block();
		}
	}
}
