package uni.oldenburg.server.agent.helper;

import uni.oldenburg.client.presenter.MainFramePresenter;
import de.novanic.eventservice.client.event.Event;
import de.novanic.eventservice.client.event.domain.DomainFactory;
import de.novanic.eventservice.service.RemoteEventServiceServlet;

@SuppressWarnings("serial")
public class EventHelper extends RemoteEventServiceServlet {	
	private static EventHelper myHelper = null;
	
	private static EventHelper getInstance() {
		if (myHelper == null) 
			myHelper = new EventHelper();
		
		return myHelper;
	}
	
	public static void addEvent(Event newEvent) {
		EventHelper.getInstance().addEvent(DomainFactory.getDomain(MainFramePresenter.DOMAIN_NAME), newEvent);
	}
}