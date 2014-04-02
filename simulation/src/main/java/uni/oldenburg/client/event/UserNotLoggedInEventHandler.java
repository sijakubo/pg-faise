package uni.oldenburg.client.event;

import com.google.gwt.event.shared.EventHandler;

/**
 * @author sijakubo
 */
public interface UserNotLoggedInEventHandler extends EventHandler {
    void onSiteCall(UserNotLoggedInEvent event);
}
