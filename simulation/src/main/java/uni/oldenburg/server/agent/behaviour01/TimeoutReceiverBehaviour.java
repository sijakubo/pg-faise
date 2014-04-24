package uni.oldenburg.server.agent.behaviour01;

import java.io.IOException;

import jade.core.Agent;
import jade.core.behaviours.SimpleBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

/**
 * behaviour that receives messages based on a template
 * during a specified timeframe until it shuts down
 * 
 * @author Matthias
 */	
@SuppressWarnings("serial")
public abstract class TimeoutReceiverBehaviour extends SimpleBehaviour {
	int timeoutMS = 0;
	long timeoutTime = 0;
	MessageTemplate mt = null;
	
	boolean reachedTimeout = false;
	private ACLMessage msg = null;
	
	boolean timeoutEventTriggered = false;
	
	public TimeoutReceiverBehaviour(Agent myAgent, int timeoutMS, MessageTemplate mt) {
		super(myAgent);
		this.timeoutMS = timeoutMS;
		this.mt = mt;
	}

	public void onStart() {
		this.reset();
		super.onStart();
	}
	
	public abstract void onMessage(ACLMessage msg);
	public abstract void onTimeout() throws IOException;

	public void action() {
		if (mt == null)
			msg = myAgent.receive();
		else
			msg = myAgent.receive(mt);
		
		if (msg != null)
			onMessage(msg);
		
		if (System.currentTimeMillis() < timeoutTime)
			block(1);
		else
			reachedTimeout = true;
	}

	public boolean done() {
		if (!timeoutEventTriggered && reachedTimeout) {
			timeoutEventTriggered = true;
			
			try {
				onTimeout();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		return reachedTimeout;
	}
	
	public void reset() {
		msg = null;
		reachedTimeout = false;
		timeoutEventTriggered = false;
		timeoutTime = timeoutMS < 0 ? 1000 : System.currentTimeMillis() + timeoutMS;
		
		super.reset();
	}	
}
