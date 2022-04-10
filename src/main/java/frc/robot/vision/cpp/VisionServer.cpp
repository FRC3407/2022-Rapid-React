#include "VisionServer.h"

#include <iostream>


VisionServer::VisionServer() :
	root = nt::NetworkTableInstance::GetDefault().GetTable("Vision Server"),
	targets = nt::NetworkTableInstance::GetDefault().GetTable("Targets"),
	cameras = root->GetSubTable("Cameras"),
	pipelines = root->GetSubTable("Pipelines"),
	active_target = root->GetEntry("Active Target"),
	num_cams = root->GetEntry("Cameras Available"),
	cam_idx = root->GetEntry("Camera Index"),
	num_pipes = root->GetEntry("Pipelines Available"),
	pipe_idx = root->GetEntry("Pipeline Index")
{
	this->root->GetEntry("Robot-CoProcessor Connected?").SetBoolean(false);
	this->cameras->AddSubTableListener(
		[this](NetworkTable* p, std::string_view n, std::shared_ptr<NetworkTable> t){
			VisionServer::updateCameras();
			this->connected = true;
			this->root->GetEntry("Robot-CoProcessor Connected?").SetBoolean(true);
		}, false
	);
	this.pipelines->AddSubTableListener(
		[this](NetworkTable* p, std::string_view n, std::shared_ptr<NetworkTable> t){
			VisionServer::updatePipelines();
			this->connected = true;
		}, false
	);
	std::cout << "VisionServer Initialized.\n";
}
VisionServer& VisionServer::Get() {
	return vsi;
}

void VisionServer::updateCameras() {
	vsi.vscameras.clear();
	for(const std::string& subtable : cameras->GetSubTables()) {
		vsi.vscameras.emplace_back(subtable);
	}
}
void VisionServer::updatePipelines() {
	vsi.vspipelines.clear();
	for(const std::string& subtable : cameras->GetSubTables()) {
		vsi.vspipelines.emplace_back(subtable);
	}
}
std::vector<VisionServer::VsCamera>& VisionServer::getCameras() { 
	if(!vsi.areCamerasUpdated()) { 
		vsi.updateCameras(); 
	}
	return vsi.vscameras; 
}
std::vector<VisionServer::VsPipeline>& VisionServer::getPipelines() { 
	if(!vsi.arePipelinesUpdated()) { 
		vsi.updatePipelines(); 
	}
	return vsi.vspipelines; 
}
VisionServer::VsCamera* VisionServer::getCamera(size_t idx) { 
	if(!vsi.areCamerasUpdated()) { 
		vsi.updateCameras(); 
	}
	return idx < vsi.vscameras.size() ? &vsi.vscameras[idx] : nullptr; 
}
VisionServer::VsCamera* VisionServer::getCamera(const std::string& name) {
	if(!vsi.areCamerasUpdated()) { 
		vsi.updateCameras(); 
	}
	for(size_t i = 0; i < vsi.vscameras.size(); i++) {
		if(vsi.vscameras[i].name == name) {
			return &vsi.vscameras[i];
		}
	}
	return nullptr;
}
int8_t VisionServer::findCameraIdx(const std::string& name) {
	for(int8_t i = 0; i < vsi.vscameras.size(); i++) {
		if(vsi.vscameras[i].name == name) {
			return i;
		}
	}
	return -1;
}
VisionServer::VsPipeline* VisionServer::getPipeline(size_t idx) { 
	if(!vsi.arePipelinesUpdated()) { 
		vsi.updatePipelines(); 
	}
	return idx < vsi.vspipelines.size() ? &vsi.vspipelines[idx] : nullptr; 
}
VisionServer::VsPipeline* VisionServer::getPipeline(const std::string& name) {
	if(!vsi.arePipelinesUpdated()) { 
		vsi.updatePipelines(); 
	}
	for(size_t i = 0; i < vsi.vspipelines.size(); i++) {
		if(vsi.vspipelines[i].name == name) {
			return &vsi.vspipelines[i];
		}
	}
	return nullptr;
}
int8_t VisionServer::findPipelineIdx(const std::string& name) {
	for(int8_t i = 0; i < vsi.vspipelines.size(); i++) {
		if(vsi.vspipelines[i].name == name) {
			return i;
		}
	}
	return -1;
}
VisionServer::VsCamera& VisionServer::getCurrentCamera() { 
	if(!vsi.areCamerasUpdated()) { 
		vsi.updateCameras(); 
	}
	return *vsi.getCamera(vsi.root->GetEntry("Camera Name").GetString("none")); 
}
VisionServer::VsPipeline& VisionServer::getCurrentPipeline() { 
	if(!vsi.arePipelinesUpdated()) { 
		vsi.updatePipelines(); 
	}
	return *vsi.getPipeline(vsi.getPipelineIdx()); 
}

bool VisionServer::applyCameraPreset(CameraPreset p) {
	bool ret = vsi.vscameras.size() > 0;
	for(size_t i = 0; i < vsi.vscameras.size(); i++) {
		ret &= vsi.vscameras[i].applyPreset(p);
	}
	return ret;
}

bool VisionServer::getIsProcessingEnabled() const {
	if(vsi.root->ContainsKey("Enable Processing")) {
		return vsi.root->GetEntry("Enable Processing").GetBoolean(true);
	}
	return false;
}
bool VisionServer::setProcessingEnabled(bool val) {
	if(vsi.root->ContainsKey("Enable Processing")) {
		return vsi.root->GetEntry("Enable Processing").SetBoolean(val);
	}
	return false;
}
bool VisionServer::toggleProcessingEnabled() {
	if(vsi.root->ContainsKey("Enable Processing")) {
		nt::NetworkTableEntry enbl = vsi.root->GetEntry("Enable Processing");
		return enbl.SetBoolean(!enbl.GetBoolean(true));
	}
	return false;
}

TargetData* VisionServer::getTargetDataIfMatching(const std::string& target) {
	if(getActiveTargetName() == target) {
		return new TargetData(std::move(getTargetData()));
	}
	return nullptr;
}

bool VisionServer::setCamera(const std::string& name) {
	std::vector<std::string> tables = vsi.cameras->GetSubTables();
	for(size_t i = 0; i < tables.size(); i++) {
		if(tables[i] == name) {
			return setCamera(i);
		}
	}
	return false;
}
bool VisionServer::incrementCamera() {
	int8_t idx = vsi.getCameraIdx();
	if(idx + 1 < vsi.numCameras()) {
		vsi.cam_idx.SetDouble(idx + 1);
		return true;
	}
	vsi.cam_idx.SetDouble(0.f);
	return false;
}
bool VisionServer::decrementCamera() {
	int8_t idx = vsi.getCameraIdx();
	if(idx - 1 >= 0) {
		vsi.cam_idx.SetDouble(idx - 1);
		return true;
	}
	vsi.cam_idx.SetDouble(vsi.numCameras() - 1);
	return true;
}
bool VisionServer::setPipeline(const std::string& name) {
	std::vector<std::string> tables = vsi.pipelines->GetSubTables();
	for(size_t i = 0; i < tables.size(); i++) {
		if(tables[i] == name) {
			return setCamera(i);
		}
	}
	return false;
}
bool VisionServer::incrementPipeline() {
	int8_t idx = vsi.getPipelineIdx();
	if(idx + 1 < vsi.numPipelines()) {
		vsi.pipe_idx.SetDouble(idx + 1);
		return true;
	}
	vsi.pipe_idx.SetDouble(0.f);
	return false;
}
bool VisionServer::decrementPipeline() {
	int8_t idx = vsi.getPipelineIdx();
	if(idx - 1 >= 0) {
		vsi.pipe_idx.SetDouble(idx - 1);
		return true;
	}
	vsi.pipe_idx.SetDouble(vsi.numPipelines() - 1);
	return false;
}

VisionServer::TargetOffset::TargetOffset(double x, double y, double z) : x(x), y(y), z(z) {}
VisionServer::TargetOffset::TargetOffset(const std::shared_ptr<nt::NetworkTable>& target) :
	x(target->GetEntry("x").GetDouble(0.f)), y(target->GetEntry("y").GetDouble(0.f)), z(target->GetEntry("z").GetDouble(0.f)) {}
VisionServer::TargetData::TargetData(double x, double y, double z, double d, double ud, double lr) : 
	pos(x, y, z), distance(d), ud(ud), lr(lr) {}
VisionServer::TargetData::TargetData(const TargetOffset& pos, double d, double ud, double lr) :
	pos(pos), distance(d), ud(ud), lr(lr) {}
VisionServer::TargetData::TargetData(const std::shared_ptr<nt::NetworkTable>& target) :
	pos(target), distance(target->GetEntry("distance").GetDouble(0.f)), 
	ud(target->GetEntry("up-down").GetDouble(0.f)), lr(target->GetEntry("left-right").GetDouble(0.f)) {}

VisionServer::VsCamera::VsCamera(const std::shared_ptr<nt::NetworkTable>& nt) {
	this->update(nt);
}
VisionServer::VsCamera::VsCamera(const std::string& name) {
	this->update(name);
}
void VisionServer::VsCamera::update(const std::shared_ptr<nt::NetworkTable>& nt) {
	this->self = nt;
	this->name = nt::NetworkTable::BasenameKey(nt->GetPath());
}
void VisionServer::VsCamera::update(const std::string& tname) {
	this->self = VisionServer::cameras->GetSubTable(tname);
	this->name = tname;
}

VisionServer::VsPipeline::VsPipeline(const std::shared_ptr<nt::NetworkTable>& nt) {
	this->update(nt);
}
VisionServer::VsPipeline::VsPipeline(const std::string& name) {
	this->update(name);
}
void VisionServer::VsPipeline::update(const std::shared_ptr<nt::NetworkTable>& nt) {
	this->self = nt;
	this->name = nt::NetworkTable::BasenameKey(nt->GetPath());
}
void VisionServer::VsPipeline::update(const std::string& tname) {
	this->self = VisionServer::cameras->GetSubTable(tname);
	this->name = tname;
}
std::vector<nt::NetworkTableEntry> VisionServer::VsPipeline::getEntries() const {
	std::vector<nt::NetworkTableEntry> entries;
	for(const std::string& key : this->self->GetKeys()) {
		entries.emplace_back(std::move(this->self->GetEntry(key)));
	}
	return entries;
}
nt::NetworkTableEntry VisionServer::VsPipeline::searchEntries(const std::string& segment) const {
	for(const std::string& key : this->self->GetKeys()) {
		if(key.find(segment) != std::string::npos) {
			return this->self->GetEntry(key);
		}
	}
	return nt::NetworkTableEntry();
}
std::vector<nt::NetworkTableEntry> VisionServer::VsPipeline::searchEntries(const std::vector<std::string>& segments) const {
	std::vector<nt::NetworkTableEntry> entries;
	for(const std::string& key : this->self->GetKeys()) {
		for(size_t i = 0; i < segments.size(); i++) {
			if(key.find(segments[i]) != std::string::npos) {
				entries.emplace_back(std::move(this->self->GetEntry(key)));
			}
		}
	}
	return entries;
}
void VisionServer::VsPipeline::searchUsableEntries() {
	for(const std::string& key : this->self->GetKeys()) {
		if(key.find("Debug")) {
			this->debug = this->self->GetEntry(key);
		}
		if(key.find("Threshold")) {
			this->debug = this->self->GetEntry(key);
		}
	}
}
bool VisionServer::VsPipeline::hasDebug() {
	if(!this->debug.Exists()) {
		this->searchUsableEntries();
		if(!this->debug.Exists()) {
			return false;
		}
	}
	return true;
}
bool VisionServer::VsPipeline::hasThreshold() {
	if(!this->thresh.Exists()) {
		this->searchUsableEntries();
		if(!this->thresh.Exists()) {
			return false;
		}
	}
	return true;
}
bool VisionServer::VsPipeline::setDebug(bool val) {
	if(!this->debug.Exists()) {
		this->searchUsableEntries();
		if(!this->debug.Exists()) {
			return false;
		}
	}
	return this->debug.SetBoolean(val);
}
bool VisionServer::VsPipeline::setThreshold(bool val) {
	if(!this->thresh.Exists()) {
		this->searchUsableEntries();
		if(!this->thresh.Exists()) {
			return false;
		}
	}
	return this->thresh.SetBoolean(val);
}