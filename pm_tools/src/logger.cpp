/*
 */
#include <pm_tools/logger.h>
#include <pm_tools/visp_tools.h>

SimplePoseLogger::SimplePoseLogger(std::string file_name, bool default_header ) {
  myfile.open((file_name + std::string(".csv")).c_str());
  if (default_header)
    setPoseHeader();
}

void SimplePoseLogger::setPoseHeader(){
  myfile << "Stamp,PoseX,PoseY,PoseZ\n";
}

void SimplePoseLogger::storePose( geometry_msgs::PoseStamped msg ){
  myfile << msg.header.stamp << ",";
  myfile << msg.pose.position.x << ",";
  myfile << msg.pose.position.y << ",";
  myfile << msg.pose.position.z << "\n";
}

SimplePoseLogger::~SimplePoseLogger(){
  myfile.close();
}

AllDataLogger::AllDataLogger(std::string file_name, std::string objectId, ros::NodeHandle & nh, bool appendMode, bool getAverages) {

  if(appendMode){
    file_.open((file_name + std::string(".csv")).c_str(), std::ios_base::app);
  }else{
    file_.open((file_name + std::string(".csv")).c_str());
  }

  if(getAverages) {
    cloudSizeSubscriber_ = nh.subscribe<std_msgs::Float32>("object/cloudSize", 1, &AllDataLogger::cloudSizeCallback,
                                                           this);
    symmetrySubscriber_ = nh.subscribe<std_msgs::Float32>("stats/symmetry/average", 1, &AllDataLogger::symmetryCallback,
                                                          this);
    estimationSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/estimation/average", 1,
                                                            &AllDataLogger::estimationCallback, this);
    backgroundSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/background/average", 1,
                                                            &AllDataLogger::backgroundCallback, this);
    filterSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/filterCloud/average", 1, &AllDataLogger::filterCallback,
                                                        this);
    loadSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/loadCloud/average", 1, &AllDataLogger::loadCallback, this);
    processSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/processCloud/average", 1,
                                                         &AllDataLogger::processCallback, this);

    modelParametersSubscriber_ = nh.subscribe<std_msgs::Float32MultiArray>("object/modelParameters/average", 1,
                                                                           &AllDataLogger::modelParametersCallback,
                                                                           this);
    poseSubscriber_ = nh.subscribe<geometry_msgs::Pose>("object/pose/average", 1, &AllDataLogger::poseCallback, this);
  }else{

  }
  aliveSubscribers_ = 9;
  objectId_ = objectId;
}

void AllDataLogger::writeRANSACCylinderHeader() {
  file_ << "Id,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Radius,Height," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataLogger::writeSQHeader() {
  file_ << "Id,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "e1,e2,a,b,c," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataLogger::writePCAHeader() {
  writeRANSACBoxHeader();
}

void AllDataLogger::writeRANSACBoxHeader() {
  file_ << "Id,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Width,Height,Depth," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataLogger::writeRANSACSphereHeader() {
  file_ << "Id,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Radius," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataLogger::writeRow() {
  while(getAliveSubscribers()>0){
    ros::spinOnce();
  }
  file_ << objectId_ << ",";
  file_ << cloudSize_ << ",";
  file_ << process_ << ",";
  file_ << load_ << ",";
  file_ << filter_ << ",";
  file_ << background_ << ",";
  file_ << estimation_ << ",";
  file_ << symmetry_ << ",";
  for(int i = 0; i < modelParameters_.size(); i++){
    file_ << modelParameters_[i] << ",";
  }
  file_ << pose_.position.x << ",";
  file_ << pose_.position.y << ",";
  file_ << pose_.position.z << ",";
  double r, p, y;
  VispTools::rpyFromQuaternion(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w, r, p, y);
  file_ << r << ",";
  file_ << p << ",";
  file_ << y << "\n";
}

int AllDataLogger::getAliveSubscribers(){
  return aliveSubscribers_;
}

void AllDataLogger::modelParametersCallback(const std_msgs::Float32MultiArrayConstPtr& m){
  for(int i=0; i<m->data.size(); i++){
    modelParameters_.push_back(m->data[i]);
  }
  modelParametersSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataLogger::cloudSizeCallback(const std_msgs::Float32ConstPtr& m){
  cloudSize_= m->data;
  cloudSizeSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataLogger::symmetryCallback(const std_msgs::Float32ConstPtr& m){
  symmetry_= m->data;
  symmetrySubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataLogger::backgroundCallback(const std_msgs::Float32ConstPtr& m){
  background_= m->data;
  backgroundSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataLogger::estimationCallback(const std_msgs::Float32ConstPtr& m){
  estimation_= m->data;
  estimationSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataLogger::filterCallback(const std_msgs::Float32ConstPtr& m){
  filter_= m->data;
  filterSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataLogger::loadCallback(const std_msgs::Float32ConstPtr& m){
  load_ = m->data;
  loadSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataLogger::processCallback(const std_msgs::Float32ConstPtr& m){
  process_ = m->data;
  processSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataLogger::poseCallback(const geometry_msgs::PoseConstPtr& m){
  pose_ = *m;
  poseSubscriber_.shutdown();
  aliveSubscribers_--;
}

AllDataLogger::~AllDataLogger() {
  file_.close();
}


