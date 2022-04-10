package frc.robot.vision.java;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;


public final class VisionServer {

	public static interface Conversion { double convert(double v); }


	private ArrayList<VsPipeline> vspipelines = new ArrayList<VsPipeline>();
	private ArrayList<VsCamera> vscameras = new ArrayList<VsCamera>();

	private boolean connected = false;

	private final NetworkTable
		root,
		targets,
		cameras,
		pipelines;
	private final NetworkTableEntry
		active_target,
		num_cams,
		cam_idx,
		num_pipes,
		pipe_idx;


	private VisionServer() {

		this.root = NetworkTableInstance.getDefault().getTable( "Vision Server" );
		this.targets = NetworkTableInstance.getDefault().getTable( "Targets" );
		this.cameras = root.getSubTable( "Cameras" );
		this.pipelines = root.getSubTable( "Pipelines" );

		this.active_target = root.getEntry( "Active Target" );
		this.num_cams = root.getEntry( "Cameras Available" );
		this.cam_idx = root.getEntry( "Camera Index" );
		this.num_pipes = root.getEntry( "Pipelines Available" );
		this.pipe_idx = root.getEntry( "Pipeline Index" );

		this.root.getEntry("Robot-CoProcessor Connected?").setBoolean(false);

		this.cameras.addSubTableListener(
			(parent, name, table) -> {
				updateCameras();
				this.connected = true;
				this.root.getEntry("Robot-CoProcessor Connected?").setBoolean(true);
			}, false
		);
		this.pipelines.addSubTableListener(
			(parent, name, table) -> {
				updatePipelines();
				this.connected = true;
			}, false
		);

		System.out.println("VisionServer Initialized.");

	}
	private static VisionServer vsi = new VisionServer();	// "VisionServer Instance"
	public static VisionServer Get() { return vsi; }



	public static NetworkTable getRoot() {
		return vsi.root;
	}
	public static NetworkTable getTargetsTable() {
		return vsi.targets;
	}
	public static NetworkTable getCamerasTable() {
		return vsi.cameras;
	}
	public static NetworkTable getPipelinesTable() {
		return vsi.pipelines;
	}



	public static boolean areCamerasUpdated() {
		return vsi.vscameras.size() == (int)vsi.num_cams.getDouble(0.0);
	}
	public static boolean arePipelinesUpdated() {
		return vsi.vspipelines.size() == (int)vsi.num_pipes.getDouble(0.0);
	}
	public static void updateCameras() {
		vsi.vscameras.clear();
		int i  = 0;
		for(String subtable : vsi.cameras.getSubTables()) {
			vsi.vscameras.add(new VsCamera(vsi.cameras.getSubTable(subtable)));
			vsi.vscameras.get(i).idx = i;
			i++;
		}
		System.out.println("Updated Cameras");
	}
	public static void updatePipelines() {
		vsi.vspipelines.clear();
		int i = 0;
		for(String subtable : vsi.pipelines.getSubTables()) {
			vsi.vspipelines.add(new VsPipeline(vsi.pipelines.getSubTable(subtable)));
			vsi.vspipelines.get(i).idx = i;
			i++;
		}
		System.out.println("Updated Pipelines");
	}
	public static ArrayList<VsCamera> getCameras() {
		return vsi.vscameras; 
	}
	public static ArrayList<VsPipeline> getPipelines() {
		return vsi.vspipelines; 
	}
	public static VsCamera getCamera(int idx) {
		return idx < vsi.vscameras.size() ? idx > 0 ? vsi.vscameras.get(idx) : null : null; 
	}
	public static VsCamera getCamera(String name) {
		for(int i = 0; i < vsi.vscameras.size(); i++) {
			if(vsi.vscameras.get(i).name.equals(name)) {
				return vsi.vscameras.get(i);
			}
		}
		return null;
	}
	public static int findCameraIdx(String name) {
		for(int i = 0; i < vsi.vscameras.size(); i++) {
			if(vsi.vscameras.get(i).name.equals(name)) {
				return i;
			}
		}
		return -1;
	}
	public static VsPipeline getPipeline(int idx) {
		return idx < vsi.vspipelines.size() ? idx >= 0 ? vsi.vspipelines.get(idx) : null : null; 
	}
	public static VsPipeline getPipeline(String name) {
		for(int i = 0; i < vsi.vspipelines.size(); i++) {
			if(vsi.vspipelines.get(i).name.equals(name)) {
				return vsi.vspipelines.get(i);
			}
		}
		return null;
	}
	public static int findPipelineIdx(String name) {	// returns -1 on failure
		for(int i = 0; i < vsi.vspipelines.size(); i++) {
			if(vsi.vspipelines.get(i).name.equals(name)) {
				return i;
			}
		}
		return -1;
	}
	public static VsCamera getCurrentCamera() {
		return getCamera(vsi.root.getEntry("Camera Name").getString("none")); 
	}
	public static VsPipeline getCurrentPipeline() {
		return getPipeline(getPipelineIdx()); 
	}

	public static boolean applyCameraPreset(CameraPreset p) {
		boolean ret = vsi.vscameras.size() > 0;
		for(VsCamera camera : vsi.vscameras) {
			ret &= camera.applyPreset(p);
		}
		return ret;
	}

	public static boolean getIsShowingStatistics() {
		return vsi.root.getEntry("Show Statistics").getBoolean(false);
	}
	public static void setStatistics(boolean val) {
		vsi.root.getEntry("Show Statistics").setBoolean(val);
	}
	public static void toggleStatistics() {
		setStatistics(!getIsShowingStatistics());
	}

	public static boolean getIsProcessingEnabled() { 
		if(vsi.root.containsKey("Enable Processing")) {
			return vsi.root.getEntry("Enable Processing").getBoolean(true);
		}
		return false;
	}
	public static boolean setProcessingEnabled(boolean val) {
		if(vsi.root.containsKey("Enable Processing")) {
			return vsi.root.getEntry("Enable Processing").setBoolean(val);
		}
		return false;
	}
	public static boolean toggleProcessingEnabled() {
		if(vsi.root.containsKey("Enable Processing")) {
			return vsi.root.getEntry("Enable Processing").setBoolean(!vsi.root.getEntry("Enable Processing").getBoolean(true));
		}
		return false;
	}

	public static boolean isConnected() {
		return vsi.connected;
	}
	public static boolean hasActiveTarget() {
		return !vsi.active_target.getString("none").equals("none");	// returns "none" on failure
	}
	public static NetworkTable getActiveTarget() {
		return vsi.targets.getSubTable(vsi.active_target.getString("none"));	// returns "none" on failure
	}
	public static String getActiveTargetName() {
		return vsi.active_target.getString("none");	// returns "none" on failure
	}
	public static double getDistance() {
		return getActiveTarget().getEntry("distance").getDouble(0.0);
	}
	public static double getThetaUD() {
		return getActiveTarget().getEntry("up-down").getDouble(0.0);
	}
	public static double getThetaLR() {
		return getActiveTarget().getEntry("left-right").getDouble(0.0);
	}
	public static TargetOffset getTargetPos() {
		return new TargetOffset(getActiveTarget());
	}
	public static TargetData getTargetData() {
		return new TargetData(getActiveTarget());
	}
	public static TargetData getTargetDataIfMatching(String target) {	// returns null on mismatch
		if(getActiveTargetName().equals(target)) {
			return getTargetData();
		}
		return null;
	}

	public static int numCameras() {
		return (int)vsi.num_cams.getDouble(0.0);	// returns 0 on failure
	}
	public static int getCameraIdx() {
		return (int)vsi.cam_idx.getDouble(-1.0);	// returns -1 on failure
	}
	public static boolean setCamera(int idx) {		// returns whether the input index was valid or not
		return idx < numCameras() && idx >= 0 && vsi.cam_idx.setDouble(idx);
	}
	public static boolean setCamera(String name) {
		int i = 0;
		for(String c : vsi.cameras.getSubTables()) {
			if(c.equals(name)) {
				//System.out.println("SetCamera: idx(" + i + "), name(" + name + ")");
				return setCamera(i);
			}
			i++;
		}
		return false;
	}
	public static boolean setCamera(VsCamera cam) {
		return setCamera(cam.name);
	}
	public static boolean incrementCamera() {
		int idx = getCameraIdx();
		if(idx + 1 < numCameras()) {
			vsi.cam_idx.setDouble(idx + 1);
			return true;
		}
		vsi.cam_idx.setDouble(0.0);	// wrap around
		return false;
	}
	public static boolean decrementCamera() {
		int idx = getCameraIdx();
		if(idx - 1 >= 0) {
			vsi.cam_idx.setDouble(idx - 1);
			return true;
		}
		vsi.cam_idx.setDouble(numCameras() - 1);	// wrap around
		return false;
	}
	public static int numPipelines() {
		return (int)vsi.num_pipes.getDouble(0.0);		// returns 0 on failure
	}
	public static int getPipelineIdx() {
		return (int)vsi.pipe_idx.getDouble(-1.0);		// returns -1 on failure
	}
	public static boolean setPipeline(int idx) {	// returns whether the input index was valid or not
		return idx < numPipelines() && idx >= 0 && vsi.pipe_idx.setDouble(idx);
	}
	public static boolean setPipeline(String name) {
		int i = 0;
		for(String p : vsi.pipelines.getSubTables()) {
			if(p.equals(name)) {
				return setPipeline(i);
			}
			i++;
		}
		return false;
	}
	public static boolean setPipeline(VsPipeline pipe) {
		return setPipeline(pipe.name);
	}
	public static boolean incrementPipeline() {
		int idx = getPipelineIdx();
		if(idx + 1 < numPipelines()) {
			vsi.pipe_idx.setDouble(idx + 1);
			return true;
		}
		vsi.pipe_idx.setDouble(0.0);	// wrap around
		return false;
	}
	public static boolean decrementPipeline() {
		int idx = getPipelineIdx();
		if(idx - 1 >= 0) {
			vsi.pipe_idx.setDouble(idx - 1);
			return true;
		}
		vsi.pipe_idx.setDouble(numPipelines() - 1);	// wrap around
		return false;
	}



	public static class CameraPreset {
		
		public final int brightness, exposure, whitebalance;

		public CameraPreset(int b, int e, int wb) {
			this.brightness = b;
			this.exposure = e;
			this.whitebalance = wb;
		}

	}
	public static interface EntryPreset { void setValue(NetworkTable nt); }
	public static abstract class EntryOption<type> implements EntryPreset {
		
		public final String entry;
		public final type value;

		public EntryOption(String n, type v) {
			this.entry = n;
			this.value = v;
		}
		//public abstract void setValue(NetworkTable nt);

	}
	public static class BooleanOption extends EntryOption<Boolean> {

		public BooleanOption(String n, Boolean v) { 
			super(n, v); 
		}
		public void setValue(NetworkTable nt) {
			if(nt.containsKey(this.entry)) {
				nt.getEntry(this.entry).setBoolean(this.value.booleanValue());
			}
		}

	}
	public static class NumericOption extends EntryOption<Double> {

		public NumericOption(String n, Double v) { 
			super(n, v); 
		}
		public void setValue(NetworkTable nt) {
			if(nt.containsKey(this.entry)) {
				nt.getEntry(this.entry).setDouble(this.value.doubleValue());
			}
		}

	}
	public static class StringOption extends EntryOption<String> {

		public StringOption(String n, String v) {
			super(n, v);
		}
		public void setValue(NetworkTable nt) {
			if(nt.containsKey(this.entry)) {
				nt.getEntry(this.entry).setString(this.value);
			}
		}

	}



	public static class VsCamera {

		private NetworkTable self;
		private String name;
		private int idx = -1;

		public void update(NetworkTable nt) {
			this.self = nt;
			this.name = NetworkTable.basenameKey(nt.getPath());
		}
		public void update(String tname) {
			this.self = VisionServer.Get().cameras.getSubTable(tname);
			this.name = tname;
		}
		public VsCamera(NetworkTable nt) { this.update(nt); }
		public VsCamera(String tname) { this.update(tname); }

		public int getIdx() {
			return this.idx;
		}
		public String getName() {
			return this.name;
		}
		public NetworkTable get() {
			return this.self;
		}

		public int getExposure() {
			return (int)this.self.getEntry("Exposure").getDouble(0.0);
		}
		public int getBrightness() {
			return (int)this.self.getEntry("Brightness").getDouble(0.0);
		}
		public int getWhiteBalance() {
			return (int)this.self.getEntry("WhiteBalance").getDouble(0.0);
		}
		public boolean setExposure(int e) {
			return this.self.getEntry("Exposure").setDouble(e);
		}
		public boolean setBrightness(int b) {
			return this.self.getEntry("Brightness").setDouble(b);
		}
		public boolean setWhiteBalance(int wb) {
			return this.self.getEntry("WhiteBalance").setDouble(wb);
		}
		public NetworkTableEntry getExposureEntry() {
			return this.self.getEntry("Exposure");
		}
		public NetworkTableEntry getBrightnessEntry() {
			return this.self.getEntry("Brightness");
		}
		public NetworkTableEntry getWhiteBalanceEntry() {
			return this.self.getEntry("WhiteBalance");
		}
		public boolean applyPreset(CameraPreset p) {
			return this.setBrightness(p.brightness) && this.setExposure(p.exposure) && this.setWhiteBalance(p.whitebalance);
		}

		public String toString() {
			return this.getClass().getName() + '(' + this.name + ")@" + Integer.toHexString(this.hashCode()) + 
				": {EX: " + this.getExposure() + ", BR: " + this.getBrightness() + ", WB: " + this.getWhiteBalance() + '}';
		}


	}
	public static class VsPipeline {

		private NetworkTable self;
		private String name;
		private int idx = -1;

		NetworkTableEntry debug = null, thresh = null;

		public void update(NetworkTable nt) {
			this.self = nt;
			this.name = NetworkTable.basenameKey(nt.getPath());
		}
		public void update(String tname) {
			this.self = VisionServer.Get().pipelines.getSubTable(tname);
			this.name = tname;
		}
		public VsPipeline(NetworkTable nt) {
			this.update(nt);
		}
		public VsPipeline(String tname) {
			this.update(tname);
		}

		public int getIdx() {
			return this.idx;
		}
		public String getName() {
			return this.name;
		}
		public NetworkTable get() {
			return this.self;
		}

		public void applyProperties(EntryPreset[] properties) {
			for(EntryPreset preset : properties) {
				preset.setValue(this.self);
			}
		}
		
		public NetworkTableEntry[] getEntries() {
			NetworkTableEntry entries[] = new NetworkTableEntry[this.self.getKeys().size()];
			int i = 0;
			for(String entry : this.self.getKeys()) {
				entries[i] = (this.self.getEntry(entry));
				i++;
			}
			return entries;
		}
		public NetworkTableEntry searchEntries(String segment) {
			String lower = segment.toLowerCase();
			for(String key : this.self.getKeys()) {
				if(key.toLowerCase().contains(lower)) {
					return this.self.getEntry(key);
				}
			}
			return null;
		}
		public NetworkTableEntry[] searchEntries(String[] segments) {
			NetworkTableEntry ret[] = new NetworkTableEntry[segments.length];
			String kbuff;
			for(String key : this.self.getKeys()) {
				kbuff = key.toLowerCase();
				for(int i = 0; i < segments.length; i++) {
					if(kbuff.contains(segments[i].toLowerCase())) {
						ret[i] = this.self.getEntry(key);
					}
				}
			}
			return ret;
		}
		public void searchUsableEntries() {
			String buff;
			for(String key : this.self.getKeys()) {
				buff = key.toLowerCase();
				if(buff.contains("debug")) {
					this.debug = this.self.getEntry(key);
				}
				if(buff.contains("threshold")) {	// else if?
					this.thresh = this.self.getEntry(key);
				}
			}
		}
		public boolean hasDebug() {
			if(this.debug == null) {
				this.searchUsableEntries();
				if(this.debug == null) {
					return false;
				}
			}
			return true;
		}
		public boolean hasThreshold() {
			if(this.thresh == null) {
				this.searchUsableEntries();
				if(this.debug == null) {
					return false;
				}
			}
			return true;
		}
		public boolean setDebug(boolean val) {
			if(this.debug == null) {
				this.searchUsableEntries();
				if(this.debug == null) {
					return false;
				}
			}
			return this.debug.setBoolean(val);
		}
		public boolean setThreshold(boolean val) {
			if(this.thresh == null) {
				this.searchUsableEntries();
				if(this.debug == null) {
					return false;
				}
			}
			return this.thresh.setBoolean(val);
		}


	}

	public static class TargetOffset {
		public double x, y, z;
		// rotation also when that gets implemented in the networktables
		public TargetOffset(double x, double y, double z) {
			this.x = x;
			this.y = y;
			this.z = z;
		}
		public TargetOffset(NetworkTable target) {
			this.x = target.getEntry("x").getDouble(0.0);
			this.y = target.getEntry("y").getDouble(0.0);
			this.z = target.getEntry("z").getDouble(0.0);
		}
	}
	public static class TargetData {
		public TargetOffset pos;
		public double distance, ud, lr;	// ud -> + : up, - : down	lr -> + : right, - : left

		public TargetData(double x, double y, double z, double d, double ud, double lr) {
			this.pos = new TargetOffset(x, y, z);
			this.distance = d;
			this.ud = ud;
			this.lr = lr;
		}
		public TargetData(TargetOffset pos, double d, double ud, double lr) {
			this.pos = pos;
			this.distance = d;
			this.ud = ud;
			this.lr = lr;
		}
		public TargetData(NetworkTable target) {
			this.pos = new TargetOffset(target);
			this.distance = target.getEntry("distance").getDouble(0.0);
			this.ud = target.getEntry("up-down").getDouble(0.0);
			this.lr = target.getEntry("left-right").getDouble(0.0);
		}
	}


}