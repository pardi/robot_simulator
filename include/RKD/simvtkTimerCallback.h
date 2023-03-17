// Author: Tommaso Pardi
// Date: 07/03/2020

#ifndef _simvtkTimerCallback_header_
#define _simvtkTimerCallback_header_

#include <vtkCommand.h>
#include <RKD/robotSimulator.h>

namespace RKD{

class robotSimulator;

class simvtkTimerCallback : public vtkCommand
{
public:
	simvtkTimerCallback() = default;

	static simvtkTimerCallback* New();

	virtual void Execute(vtkObject*, unsigned long, void*);

	void Stop();

	int timerId_{-1};

	robotSimulator* robSimObj_;
	int TimerCount_;
	int type_ = 0;
	bool stop_flag_{false};

};
}

#endif