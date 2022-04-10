#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <vector>
#include <string>


class VisionServer {
public:

	typedef double(*conversion_t)(double);

	struct CameraPreset;
	struct TargetOffset;
	struct TargetData;
	class VsCamera;
	class VsPipeline;

	const std::shared_ptr<nt::NetworkTable>
		root,
		targets,
		cameras,
		pipelines;
	const nt::NetworkTableEntry
		active_target,
		num_cams,
		cam_idx,
		num_pipes,
		pipe_idx;

	static VisionServer& Get();
	static inline bool isConnected() { return vsi.connected; }

	static inline const std::shared_ptr<nt::NetworkTable>& getRoot() { return vsi.root; }
	static inline const std::shared_ptr<nt::NetworkTable>& getTargetsTable() { return vsi.targets; }
	static inline const std::shared_ptr<nt::NetworkTable>& getCamerasTable() { return vsi.cameras; }
	static inline const std::shared_ptr<nt::NetworkTable>& getPipelinesTable() { return vsi.pipelines; }


	static inline bool areCamerasUpdated() { return vsi.vscameras.size() == (size_t)vsi.num_cams.GetDouble(0.0); }
	static inline bool arePipelinesUpdated() { return vsi.vspipelines.size() == (size_t)vsi.num_pipes.GetDouble(0.0); }
	static void updateCameras();
	static void updatePipelines();
	static std::vector<VsCamera>& getCameras();
	static std::vector<VsPipeline>& getPipelines();
	static VsCamera* getCamera(size_t idx);	// returns nullptr on oob error
	static VsCamera* getCamera(const std::string& name);
	static int8_t findCameraIdx(const std::string& name);
	static VsPipeline* getPipeline(size_t idx);	// returns nullptr on oob error
	static VsPipeline* getPipeline(const std::string& name);
	static int8_t findPipelineIdx(const std::string& name);
	static VsCamera& getCurrentCamera();	// could possibly segfault if the index isn't up to date, but this is unlikely
	static VsPipeline& getCurrentPipeline();	//^

	static bool applyCameraPreset(CameraPreset p);

	static inline bool getIsShowingStatistics() { return vsi.root->GetEntry("Show Statistics").GetBoolean(false); }
	static inline void setStatistics(bool val) { vsi.root->GetEntry("Show Statistics").SetBoolean(val); }
	static inline void toggleStatistics() { vsi.setStatistics(!vsi.getIsShowingStatistics()); }

	static bool getIsProcessingEnabled();
	static bool setProcessingEnabled(bool val);
	static bool toggleProcessingEnabled();

	static inline bool hasActiveTarget() { return vsi.active_target.GetString("none") != "none"; }
	static inline std::shared_ptr<nt::NetworkTable> getActiveTarget() { return vsi.targets->GetSubTable(vsi.active_target.GetString("none")); }
	static inline std::string getActveTargetName() { return vsi.active_target.GetString("none"); }
	static inline double getDistance() { return vsi.getActiveTarget()->GetEntry("distance").GetDouble(0.f); }
	static inline double getThetaUD() { return vsi.getActiveTarget()->GetEntry("up-down").GetDouble(0.f); }
	static inline double getThetaLR() { return vsi.getActiveTarget()->GetEntry("left-right").GetDouble(0.f); }
	static inline TargetOffset getTargetPos() { return TargetOffset(vsi.getActiveTarget()); }
	static inline TargetData getTargetData() { return TargetData(vsi.getActiveTarget()); }
	static TargetData* getTargetDataIfMatching(const std::string& target);	// heap allocated TargetData or nullptr on mismatch

	static inline uint8_t numCameras() { return vsi.num_cams.GetDouble(0.f); }
	static inline int8_t getCameraIdx() { return vsi.cam_idx.GetDouble(-1.f); }
	static inline bool setCamera(uint8_t idx) { return idx < numCameras() && vsi.cam_idx.SetDouble(idx); }
	static bool setCamera(const std::string& name);
	static inline bool setCamera(const VsCamera cam) { return setCamera(cam.name); }
	static bool incrementCamera();
	static bool decrementCamera();

	static inline uint8_t numPipelines() { return vsi.num_pipes.GetDouble(0.f); }
	static inline int8_t getPipelineIdx() { return vsi.pipe_idx.GetDouble(-1.f); }
	static inline bool setPipeline(uint8_t idx) { return idx < numPipelines() && vsi.pipe_idx.SetDouble(idx); }
	static bool setPipeline(const std::string& name);
	static inline bool setPipeline(const VsPipeline& pipe) { return setPipeline(pipe.name); }
	static bool incrementPipeline();
	static bool decrementPipeline();



	struct CameraPreset {
		const int16_t brightness, exposure, whitebalance;

		CameraPreset();
		CameraPreset(int16_t b, int16_t e, int16_t w);
	}

	struct TargetOffset {
		const double x, y, z;

		TargetOffset(double x, double y, double z);
		TargetOffset(const std::shared_ptr<nt::NetworkTable>& target);
	};
	struct TargetData {
		const TargetOffset pos;
		const double distance, ud, lr;

		TargetData(double x, double y, double z, double d, double ud, double lr);
		TargetData(const TargetOffset& pos, double d, double ud, double lr);
		TargetData(const std::shared_ptr<nt::NetworkTable>& target);
	};

	class VsCamera {
		friend class VisionServer;
	public:
		void update(const std::shared_ptr<nt::NetworkTable>& nt);
		void update(const std::string& tname);

		VsCamera(const std::shared_ptr<nt::NetworkTable>& nt);
		VsCamera(const std::string& name);

		inline const std::shared_ptr<nt::NetworkTable>& get() const { return this->self; }
		inline const std::string& getName() const { return this->name; }
		inline const int8_t getIdx() const { return this->idx; }

		inline int8_t getExposure() const { return this->self->GetEntry("Exposure").GetDouble(0.f); }
		inline int8_t getBrightness() const { return this->self->GetEntry("Brightness").GetDouble(0.f); }
		inline int16_t getWhiteBalance() const { return this->self->GetEntry("WhiteBalance").GetDouble(0.f); }
		inline bool setExposure(int8_t e) { return this->self->GetEntry("Exposure").SetDouble(e); }
		inline bool setBrightness(int8_t b) { return this->self->GetEntry("Brightness").SetDouble(b); }
		inline bool setWhiteBalance(int16_t wb) { return this->self->GetEntry("WhiteBalance").SetDouble(wb); }
		inline nt::NetworkTableEntry getExposureEntry() const { return this->self->GetEntry("Exposure"); }
		inline nt::NetworkTableEntry getBrightnessEntry() const { return this->self->GetEntry("Brightness"); }
		inline nt::NetworkTableEntry getWhiteBalanceEntry() const { return this->self->GetEntry("WhiteBalance"); }
		inline bool applyPreset(CameraPreset p) {
			return this->setBrightness(p.brightness) && this->setExposure(p.exposure) && this->setWhiteBalance(p.whiteBalance);
		}

	private:
		std::shared_ptr<nt::NetworkTable> self;
		std::string name;
		int8_t idx{-1};

	};
	class VsPipeline {
		friend class VisionServer;
	public:
		void update(const std::shared_ptr<nt::NetworkTable>& nt);
		void update(const std::string& tname);

		VsPipeline(const std::shared_ptr<nt::NetworkTable>& nt);
		VsPipeline(const std::string& name);

		inline const std::shared_ptr<nt::NetworkTable>& get() const { return this->self; }
		inline const std::string& getName() const { return this->name; }
		inline const int8_t getIdx() const { return this->idx; }

		std::vector<nt::NetworkTableEntry> getEntries() const;
		nt::NetworkTableEntry searchEntries(const std::string& segment) const;
		std::vector<nt::NetworkTableEntry> searchEntries(const std::vector<std::string>& segments) const;
		void searchUsableEntries();
		bool hasDebug();
		bool hasThreshold();
		bool setDebug(bool val);
		bool setThreshold(bool val);

	private:
		std::shared_ptr<nt::NetworkTable> self;
		std::string name;
		int8_t idx{-1};
		nt::NetworkTableEntry debug, thresh;

	};

private:
	VisionServer();

	static VisionServer vsi;

	std::vector<VsCamera> vscameras;
	std::vector<VsPipeline> vspipelines;
	bool connected = {false};


};