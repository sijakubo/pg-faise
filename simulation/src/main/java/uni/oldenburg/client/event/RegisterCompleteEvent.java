package uni.oldenburg.client.event;

import com.google.gwt.event.shared.GwtEvent;

/**
 * @author sijakubo
 */
public class RegisterCompleteEvent extends GwtEvent<RegisterCompleteEventHandler> {
   public static Type<RegisterCompleteEventHandler> TYPE = new Type<RegisterCompleteEventHandler>();

   @Override
   public com.google.gwt.event.shared.GwtEvent.Type<RegisterCompleteEventHandler> getAssociatedType() {
      return TYPE;
   }

   @Override
   protected void dispatch(RegisterCompleteEventHandler handler) {
      handler.onRegister(this);
   }
}
