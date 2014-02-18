package uni.oldenburg.client.event;

import com.google.gwt.event.shared.GwtEvent;

public class CallRegisterEvent extends GwtEvent<CallRegisterEventHandler> {
	public static Type<CallRegisterEventHandler> TYPE = new Type<CallRegisterEventHandler>();

	@Override
	public com.google.gwt.event.shared.GwtEvent.Type<CallRegisterEventHandler> getAssociatedType() {
		return TYPE;
	}

	@Override
	protected void dispatch(CallRegisterEventHandler handler) {
		handler.onRegisterCall(this);
	}
		
}
