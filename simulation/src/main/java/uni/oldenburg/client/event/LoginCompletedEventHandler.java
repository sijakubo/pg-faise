package uni.oldenburg.client.event;

import com.google.gwt.event.shared.EventHandler;

public interface LoginCompletedEventHandler extends EventHandler{
	void onLogin(LoginCompletedEvent event);	
}
