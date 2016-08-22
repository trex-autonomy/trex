#ifndef H_ActionAdapter
#define H_ActionAdapter

#include "Adapter.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Domains.hh"


namespace TREX {

  template <class Goal, class State, class Feedback> class ActionAdapter : public Adapter {
  public:
    
  ActionAdapter(const LabelStr& agentName, const TiXmlElement& configData)
    : Adapter(agentName, configData, 1),
      timelineName(extractData(configData, "timelineName").toString()),
      timelineType(extractData(configData, "timelineType").toString()),
      inactivePredicate(timelineType + ".Inactive"), 
      activePredicate(timelineType + ".Active"),
      is_active(false), lastPublished(-1), lastUpdated(-2) {
    }
   
    virtual ~ActionAdapter()
      {    
      }

  protected:

    /**
     * @brief Handle state update message from the controller. This will fill the _state_msg parameters and
     * also handle the interpretation of the control flags to trigger state updates
     */
    virtual void handleCallback(const State &state){
      // If we have already changed the state in this tick, we do not want to over-ride that. This will ensure we do not miss a state change
      // where the goal to move is accompished almost instantly, for example if the robot is already at the goal.
      if(lastUpdated == getCurrentTick())
	return;

      bool nowActive = isActiveObs(state);

      if(nowActive != is_active) {
	// Copy observation
	//_observationLock.lock();
	_observation = state;

	// Update marker to prevent overwrite
	lastUpdated = getCurrentTick();

	TREX_INFO("ros:debug:synchronization", nameString() << "Received transition to " << nowActive);
	
	//_observationLock.unlock();

      } else {
      }
    }

    Observation* getObservation(){

      TREX_INFO("ros:debug:synchronization", nameString() << "First call - before checking flag. " <<
		"Last updated at " << lastUpdated << " and last published " <<  lastPublished);

      if(((int) lastUpdated) == lastPublished)
	return NULL;
     
      ObservationByValue* obs = NULL;

      //_observation.lock();

      if((!isActiveObs(_observation) && is_active) || lastUpdated == 0){
	//TREX_INFO("ros:debug:synchronization", nameString() << "Transitioning INACTIVE with status=" << 
	//	  LabelStr(getResultStatus(_observation).getSingletonValue()).toString());
	
	obs = new ObservationByValue(timelineName, inactivePredicate);

	fillInactiveObservationParameters(_observation.feedback, obs);


	is_active = false;
      }
      else if(isActiveObs(_observation) && !is_active){
	TREX_INFO("ros:debug:synchronization", nameString() << "Transitioning ACTIVE");

	obs = new ObservationByValue(timelineName, activePredicate);

	fillActiveObservationParameters(_observation.goal, obs);

	is_active = true;
      } else {
      }

      //_observation.unlock();

      lastPublished = lastUpdated;

      return obs;
    }

    /**
     * The goal can be enabled or disabled.
     * The predicate can be active or inactive
     */
    bool dispatchRequest(const TokenId& goal, bool enabled){
      bool enableController = enabled;

      // If the request to move into the inactive state, then evaluate the time bound and only process
      // if it is a singleton
      if(goal->getPredicateName() != activePredicate){
	if(goal->start()->lastDomain().getUpperBound() > getCurrentTick())
	  return false;

	// If already inactive, there is nothing to be done. This can occur if the action activates and succeeds immediately.
	if(!isActive()){
	  TREX_INFO("ros:debug:dispatching", nameString().c_str() << "No need to dispatch " << goal->toString());
	  return true;
	}

	enableController = false;
      }

      // If we are disabling and it is already active then 
      // Set the goal and its frame

      TREX_INFO("ros:debug:dispatching",  
		nameString().c_str() << "FIXME" << " WITH "  << goal->toLongString());

      // If this is a request to activate the action, send it. However, if it is a request to deactivate the action we need only send it
      // if the action is currently active
      if(enableController){
	Goal goal_msg;
	fillDispatchParameters(goal_msg, goal);
	publishGoal(goal_msg);
      }
      else {
	preempt();
      }

      return true;
    }

    bool handleRequest(const TokenId& goal){
      return dispatchRequest(goal, true);
    }

    bool synchronize(){
      TREX_INFO("ros:debug:synchronization", nameString() << "Synchronizing");
      // Derived class will populate actual observations
      Observation* obs = NULL;
      obs = getObservation();
      
      if(obs != NULL){
	TREX_INFO("ros:info", nameString() << "Found observation:" << obs->toString());
	sendNotify(*obs);
	delete obs;
      }
      
      return true;
    }

    virtual void publishGoal(const Goal& msg) {}

    virtual void preempt() {}

    virtual void fillActiveObservationParameters(Goal& msg, ObservationByValue* obs){}

    virtual void fillInactiveObservationParameters(const Feedback& msg, ObservationByValue* obs){}

    virtual void fillDispatchParameters(Goal& msg, const TokenId& goalToken){}

    virtual bool isActiveObs(const State &state){ return false; } 

    bool isActive() const {return is_active;}

    bool succeeded() const {return _observation.status.value == _observation.status.SUCCESS;}

  private:
    State _observation; /*!< This message is where we copy the state update on a transition */
    //boost::mutex _observationLock;
    const std::string timelineName;
    const std::string timelineType;
    const LabelStr inactivePredicate;
    const LabelStr activePredicate;
    bool is_active;
    int lastPublished;
    TICK lastUpdated;
  };
}
#endif
