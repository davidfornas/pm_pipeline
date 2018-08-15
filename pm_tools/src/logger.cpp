/*
 */
#include <pm_tools/logger.h>
#include <pm_tools/visp_tools.h>
#include <pm_tools/timing.h>

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

AllDataSingleLogger::AllDataSingleLogger(std::string file_name, std::string objectId, ros::NodeHandle & nh, bool appendMode) {

  if (appendMode) {
    file_.open((file_name + std::string(".csv")).c_str(), std::ios_base::app);
  } else {
    file_.open((file_name + std::string(".csv")).c_str());
  }

  cloudSizeSubscriber_ = nh.subscribe<std_msgs::Float32>("object/cloudSize/average", 1, &AllDataSingleLogger::cloudSizeCallback,
                                                         this);
  symmetrySubscriber_ = nh.subscribe<std_msgs::Float32>("stats/symmetry/average", 1, &AllDataSingleLogger::symmetryCallback,
                                                        this);
  estimationSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/estimation/average", 1,
                                                          &AllDataSingleLogger::estimationCallback, this);
  backgroundSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/background/average", 1,
                                                          &AllDataSingleLogger::backgroundCallback, this);
  filterSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/filterCloud/average", 1, &AllDataSingleLogger::filterCallback,
                                                      this);
  loadSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/loadCloud/average", 1, &AllDataSingleLogger::loadCallback, this);
  processSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/processCloud/average", 1,
                                                       &AllDataSingleLogger::processCallback, this);

  modelParametersSubscriber_ = nh.subscribe<std_msgs::Float32MultiArray>("object/modelParameters/average", 1,
                                                                         &AllDataSingleLogger::modelParametersCallback,
                                                                         this);
  poseSubscriber_ = nh.subscribe<geometry_msgs::Pose>("object/pose/average", 1, &AllDataSingleLogger::poseCallback, this);

  aliveSubscribers_ = 9;
  objectId_ = objectId;
}

void AllDataSingleLogger::writeRANSACCylinderHeader() {
  file_ << "Id,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Radius,Height," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataSingleLogger::writeSQHeader() {
  file_ << "Id,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "e1,e2,a,b,c," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataSingleLogger::writePCAHeader() {
  writeRANSACBoxHeader();
}

void AllDataSingleLogger::writeRANSACBoxHeader() {
  file_ << "Id,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Width,Height,Depth," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataSingleLogger::writeRANSACSphereHeader() {
  file_ << "Id,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Radius," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataSingleLogger::writeRow() {
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

int AllDataSingleLogger::getAliveSubscribers(){
  return aliveSubscribers_;
}

void AllDataSingleLogger::modelParametersCallback(const std_msgs::Float32MultiArrayConstPtr& m){
  for(int i=0; i<m->data.size(); i++){
    modelParameters_.push_back(m->data[i]);
  }
  modelParametersSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataSingleLogger::cloudSizeCallback(const std_msgs::Float32ConstPtr& m){
  cloudSize_= m->data;
  cloudSizeSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataSingleLogger::symmetryCallback(const std_msgs::Float32ConstPtr& m){
  symmetry_= m->data;
  symmetrySubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataSingleLogger::backgroundCallback(const std_msgs::Float32ConstPtr& m){
  background_= m->data;
  backgroundSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataSingleLogger::estimationCallback(const std_msgs::Float32ConstPtr& m){
  estimation_= m->data;
  estimationSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataSingleLogger::filterCallback(const std_msgs::Float32ConstPtr& m){
  filter_= m->data;
  filterSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataSingleLogger::loadCallback(const std_msgs::Float32ConstPtr& m){
  load_ = m->data;
  loadSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataSingleLogger::processCallback(const std_msgs::Float32ConstPtr& m){
  process_ = m->data;
  processSubscriber_.shutdown();
  aliveSubscribers_--;
}

void AllDataSingleLogger::poseCallback(const geometry_msgs::PoseConstPtr& m){
  pose_ = *m;
  poseSubscriber_.shutdown();
  aliveSubscribers_--;
}

AllDataSingleLogger::~AllDataSingleLogger() {
  file_.close();
}

// MULTIPLE ============================================================================  //


AllDataMultiLogger::AllDataMultiLogger(std::string file_name, std::string objectId, ros::NodeHandle & nh, int mode)
        : objectId_(objectId) {
  switch(mode) {
    case 1:
      writeRANSACCylinderHeader();
      break;
    case 2:
      writeRANSACBoxHeader();
      break;
    case 3:
      writeRANSACSphereHeader();
      break;
    case 4:
      writeSQHeader();
      break;
    case 5:
      writePCAHeader();
      break;
    default:
      break;
  }
  timerStarted_ = false;
  file_.open((file_name + std::string(".csv")).c_str());

  cloudSizeSubscriber_ = nh.subscribe<std_msgs::Float32>("object/cloudSize", 1, &AllDataMultiLogger::cloudSizeCallback,
                                                         this);
  symmetrySubscriber_ = nh.subscribe<std_msgs::Float32>("stats/symmetry", 1, &AllDataMultiLogger::symmetryCallback,
                                                        this);
  estimationSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/estimation", 1,
                                                          &AllDataMultiLogger::estimationCallback, this);
  backgroundSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/background", 1,
                                                          &AllDataMultiLogger::backgroundCallback, this);
  filterSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/filterCloud", 1, &AllDataMultiLogger::filterCallback,
                                                      this);
  loadSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/loadCloud", 1, &AllDataMultiLogger::loadCallback, this);
  processSubscriber_ = nh.subscribe<std_msgs::Float32>("stats/processCloud", 1,
                                                       &AllDataMultiLogger::processCallback, this);

  modelParametersSubscriber_ = nh.subscribe<std_msgs::Float32MultiArray>("object/modelParameters", 1,
                                                                         &AllDataMultiLogger::modelParametersCallback,
                                                                         this);
  poseSubscriber_ = nh.subscribe<geometry_msgs::Pose>("object/pose", 1, &AllDataMultiLogger::poseCallback, this);
}

void AllDataMultiLogger::writeRANSACCylinderHeader() {
  file_ << "Id,TimeSecs,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Radius,Height," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataMultiLogger::writeSQHeader() {
  file_ << "Id,TimeSecs,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "e1,e2,a,b,c," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataMultiLogger::writePCAHeader() {
  writeRANSACBoxHeader();
}

void AllDataMultiLogger::writeRANSACBoxHeader() {
  file_ << "Id,TimeSecs,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Width,Height,Depth," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataMultiLogger::writeRANSACSphereHeader() {
  file_ << "Id,TimeSecs,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Radius," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataMultiLogger::writeRow() {
  double time;
  if(!timerStarted_){
    time = 0;
    timerStarted_ = true;
    timer.resetTimer();
  }else{
    time = timer.getTotalTime();
  }
  file_ << objectId_ << ",";
  file_ << time << ",";
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

void AllDataMultiLogger::modelParametersCallback(const std_msgs::Float32MultiArrayConstPtr& m){
  for(int i=0; i<m->data.size(); i++){
    modelParameters_.push_back(m->data[i]);
  }
  modelParametersSubscriber_.shutdown();
}

void AllDataMultiLogger::cloudSizeCallback(const std_msgs::Float32ConstPtr& m){
  cloudSize_= m->data;
}

void AllDataMultiLogger::symmetryCallback(const std_msgs::Float32ConstPtr& m){
  symmetry_= m->data;
}

void AllDataMultiLogger::backgroundCallback(const std_msgs::Float32ConstPtr& m){
  background_= m->data;
}

void AllDataMultiLogger::estimationCallback(const std_msgs::Float32ConstPtr& m){
  estimation_= m->data;
}

void AllDataMultiLogger::filterCallback(const std_msgs::Float32ConstPtr& m){
  filter_= m->data;
}

void AllDataMultiLogger::loadCallback(const std_msgs::Float32ConstPtr& m){
  load_ = m->data;
}

void AllDataMultiLogger::processCallback(const std_msgs::Float32ConstPtr& m){
  process_ = m->data;
}

void AllDataMultiLogger::poseCallback(const geometry_msgs::PoseConstPtr& m){
  pose_ = *m;
  writeRow();
}

AllDataMultiLogger::~AllDataMultiLogger() {
  file_.close();
}