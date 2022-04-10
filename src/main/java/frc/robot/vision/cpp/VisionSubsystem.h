#pragma once

#include "./VisionServer.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>


namespace VisionSubsystem {

	class InstantGlobal : public frc2::CommandHelper<frc2::CommandBase, InstantGlobal> {
	public:
		InstantGlobal() = default;
		bool IsFinished() override { return true; }
		bool RunsWhenDisabled() override { return true; }
	}

	class IncrementCamera : public InstantGlobal {
	public:
		IncrementCamera() = default;
		inline void Initialize() override { VisionServer::incrementCamera(); }
	};
	class DecrementCamera : public InstantGlobal {
	public:
		DecrementCamera() = default;
		inline void Initialize() override { VisionServer::decrementCamera(); }
	};
	class IncrementPipeline : public InstantGlobal {
	public:
		IncrementPipeline() = default;
		inline void Initialize() override { VisionServer::incrementPipeline(); }
	};
	class DecrementPipeline : public InstantGlobal {
	public:
		DecrementPipeline() = default;
		inline void Initialize() override { VisionServer::decrementPipeline(); }
	};
	class ToggleStatistics : public InstantGlobal {
	public:
		ToggleStatistics() = default;
		inline void Initialize() override { VisionServer::toggleStatistics(); }
	};
	class ToggleProcessing : public InstantGlobal {
	public:
		TogglePipeline() = default;
		inline void Initialize() override { VisionServer::toggleProcessingEnabled(); }
	};


}