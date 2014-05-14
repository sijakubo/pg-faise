#include "AgentRTE/AgentRTE.h"

static void dummyAgent_init(){
	printf("Dummy Agent Init");
}
static void dummyAgent_main(){
	printf("Dummy Agent Main");
}
static void dummyAgent_kill(){
	printf("Dummy Agent Kill");
}

const struct AgentCallbacks dummyAgentCallbacks = {dummyAgent_init, dummyAgent_main, dummyAgent_kill};
