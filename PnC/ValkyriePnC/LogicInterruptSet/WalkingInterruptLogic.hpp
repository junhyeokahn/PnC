#pragma once
#include <PnC/InterruptLogic.hpp>

// Forward Declare Control Architecture
class ValkyrieControlArchitecture;

class WalkingInterruptLogic : public InterruptLogic {
public:
	WalkingInterruptLogic(ValkyrieControlArchitecture* val_ctrl_arch_);
	~WalkingInterruptLogic();

	void processInterrupts();

    ValkyrieControlArchitecture* val_ctrl_arch_;
};