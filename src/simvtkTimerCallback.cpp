#include <RKD/simvtkTimerCallback.h>

using namespace RKD;

std::shared_ptr<simvtkTimerCallback> simvtkTimerCallback::New(){
	
	std::shared_ptr<simvtkTimerCallback> cb = new simvtkTimerCallback();
	
	cb->TimerCount_ = 0;

	return cb;
}

void simvtkTimerCallback::Execute(vtkObject* caller, unsigned long eventId, void* vtkNotUsed(callData)){

	if (vtkCommand::TimerEvent == eventId)
		++TimerCount_;

	robSimObj_->getCMDFromClient();
	
	if (stop_flag_){

		std::cout << "[ERROR] Stop vtkTimerCallback" << std::endl;

		vtkRenderWindowInteractor* iren = dynamic_cast<vtkRenderWindowInteractor*>(caller);

		if (timerId_ > -1)
			iren->DestroyTimer(timerId_);	
	}
}

void simvtkTimerCallback::Stop(){
	stop_flag_ = true;
}