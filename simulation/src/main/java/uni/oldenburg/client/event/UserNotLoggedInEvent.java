package uni.oldenburg.client.event;

import com.google.gwt.event.shared.GwtEvent;

/**
 * @author sijakubo
 */
public class UserNotLoggedInEvent extends GwtEvent<UserNotLoggedInEventHandler> {
    public static Type<UserNotLoggedInEventHandler> TYPE = new Type<UserNotLoggedInEventHandler>();

    @Override
    public Type<UserNotLoggedInEventHandler> getAssociatedType() {
        return TYPE;
    }

    @Override
    protected void dispatch(UserNotLoggedInEventHandler handler) {
        handler.onSiteCall(this);
    }
}
