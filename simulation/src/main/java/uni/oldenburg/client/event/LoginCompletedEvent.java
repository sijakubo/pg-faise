package uni.oldenburg.client.event;

import com.google.gwt.event.shared.GwtEvent;


public class LoginCompletedEvent extends GwtEvent<LoginCompletedEventHandler> {
  public static Type<LoginCompletedEventHandler> TYPE = new Type<LoginCompletedEventHandler>();
  
  @Override
  public com.google.gwt.event.shared.GwtEvent.Type<LoginCompletedEventHandler> getAssociatedType() {
    return TYPE;
  }

  @Override
  protected void dispatch(LoginCompletedEventHandler handler) {
	 handler.onLogin(this);
	
  }


}
