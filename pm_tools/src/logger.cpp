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

AllDataSingleLogger::AllDataSingleLogger(std::string file_name, std::string objectId, ros::NodeHandle & nH, bool appendMode)
        : nh(nH), alive_originalCloudSizeSubscriber_(false), alive_filteredCloudSizeSubscriber_(false),
          alive_noBackgroundcloudSizeSubscriber_(false), alive_cloudSizeSubscriber_(false), alive_symmetrySubscriber_(false),
          alive_estimationSubscriber_(false), alive_backgroundSubscriber_(false), alive_filterSubscriber_(false),
          alive_loadSubscriber_(false), alive_processSubscriber_(false), alive_modelParametersSubscriber_(false),
          alive_poseSubscriber_(false)
{
  if (appendMode) {
    file_.open((file_name + std::string(".csv")).c_str(), std::ios_base::app);
  } else {
    file_.open((file_name + std::string(".csv")).c_str());
  }

  objectId_ = objectId;
  subscribeToAll();
}

void AllDataSingleLogger::subscribeToAll(){
  cloudSizeSubscriber_ = nh.subscribe<std_msgs::Float32>("object/cloudSize/average", 1, &AllDataSingleLogger::cloudSizeCallback,
                                                         this);
  originalCloudSizeSubscriber_ = nh.subscribe<std_msgs::Float32>("object/originalCloudSize/average", 1, &AllDataSingleLogger::originalCloudSizeCallback,
                                                                 this);
  filteredCloudSizeSubscriber_ = nh.subscribe<std_msgs::Float32>("object/filteredCloudSize/average", 1, &AllDataSingleLogger::filteredCloudSizeCallback,
                                                                 this);
  noBackgroundcloudSizeSubscriber_ = nh.subscribe<std_msgs::Float32>("object/noBackgroundCloudSize/average", 1, &AllDataSingleLogger::noBackgroundCloudSizeCallback,
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
}

void AllDataSingleLogger::writeRANSACCylinderHeader() {
  file_ << "Id,OriginalCloudSize,FilteredCloudSize,NoBackgroundCloudSize,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Radius,Height," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataSingleLogger::writeSQHeader() {
  file_ << "Id,OriginalCloudSize,FilteredCloudSize,NoBackgroundCloudSize,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "e1,e2,a,b,c," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataSingleLogger::writePCAHeader() {
  writeRANSACBoxHeader();
}

void AllDataSingleLogger::writeRANSACBoxHeader() {
  file_ << "Id,OriginalCloudSize,FilteredCloudSize,NoBackgroundCloudSize,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Width,Height,Depth," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataSingleLogger::writeRANSACSphereHeader() {
  file_ << "Id,OriginalCloudSize,FilteredCloudSize,NoBackgroundCloudSize,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Radius," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataSingleLogger::writeRow() {
  while(!areAllSubscribersAlive()){
    ros::spinOnce();
  }
  ROS_INFO_STREAM("Average for object " << objectId_ << "SIZE: " << modelParameters_.size());
  file_ << objectId_ << ",";
  file_ << originalCloudSize_ << ",";
  file_ << filteredCloudSize_ << ",";
  file_ << noBackgroundCloudSize_ << ",";
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

bool AllDataSingleLogger::areAllSubscribersAlive(){
  return  alive_originalCloudSizeSubscriber_ && alive_filteredCloudSizeSubscriber_  && alive_noBackgroundcloudSizeSubscriber_
          && alive_cloudSizeSubscriber_ && alive_symmetrySubscriber_ && alive_estimationSubscriber_ && alive_backgroundSubscriber_
          && alive_filterSubscriber_ && alive_loadSubscriber_ && alive_processSubscriber_
          && alive_modelParametersSubscriber_ && alive_poseSubscriber_;
}

void AllDataSingleLogger::modelParametersCallback(const std_msgs::Float32MultiArrayConstPtr& m){
  if(modelParameters_.size() == 0) {
    for (int i = 0; i < m->data.size(); i++) {
      modelParameters_.push_back(m->data[i]);
    }
  }
  else{
    for (int i = 0; i < m->data.size(); i++) {
      modelParameters_[i] = m->data[i];
    }
  }
  alive_modelParametersSubscriber_ = true;
}

void AllDataSingleLogger::cloudSizeCallback(const std_msgs::Float32ConstPtr& m){
  cloudSize_= m->data;
  alive_cloudSizeSubscriber_ = true;
}

void AllDataSingleLogger::originalCloudSizeCallback(const std_msgs::Float32ConstPtr& m){
  originalCloudSize_= m->data;
  alive_originalCloudSizeSubscriber_ = true;
}
void AllDataSingleLogger::filteredCloudSizeCallback(const std_msgs::Float32ConstPtr& m){
  filteredCloudSize_= m->data;
  alive_filteredCloudSizeSubscriber_ = true;
}

void AllDataSingleLogger::noBackgroundCloudSizeCallback(const std_msgs::Float32ConstPtr& m){
  noBackgroundCloudSize_= m->data;
  alive_noBackgroundcloudSizeSubscriber_ = true;
}

void AllDataSingleLogger::symmetryCallback(const std_msgs::Float32ConstPtr& m){
  symmetry_= m->data;
  alive_symmetrySubscriber_ = true;
}

void AllDataSingleLogger::backgroundCallback(const std_msgs::Float32ConstPtr& m){
  background_= m->data;
  alive_backgroundSubscriber_ = true;

}

void AllDataSingleLogger::estimationCallback(const std_msgs::Float32ConstPtr& m){
  estimation_= m->data;
  alive_estimationSubscriber_ = true;
}

void AllDataSingleLogger::filterCallback(const std_msgs::Float32ConstPtr& m){
  filter_= m->data;
  alive_filterSubscriber_ = true;
}

void AllDataSingleLogger::loadCallback(const std_msgs::Float32ConstPtr& m){
  load_ = m->data;
  alive_loadSubscriber_ = true;
}

void AllDataSingleLogger::processCallback(const std_msgs::Float32ConstPtr& m){
  process_ = m->data;
  alive_processSubscriber_ = true;
}

void AllDataSingleLogger::poseCallback(const geometry_msgs::PoseConstPtr& m){
  pose_ = *m;
  alive_poseSubscriber_ = true;
}

AllDataSingleLogger::~AllDataSingleLogger() {
  file_.close();
}

// MULTIPLE ============================================================================  //


AllDataMultiLogger::AllDataMultiLogger(std::string file_name, std::string objectId, ros::NodeHandle & nH, int mode)
        : objectId_(objectId), nh(nH), alive_originalCloudSizeSubscriber_(false), alive_filteredCloudSizeSubscriber_(false),
          alive_noBackgroundCloudSizeSubscriber_(false), alive_cloudSizeSubscriber_(false), alive_symmetrySubscriber_(false),
          alive_estimationSubscriber_(false), alive_backgroundSubscriber_(false), alive_filterSubscriber_(false),
          alive_loadSubscriber_(false), alive_processSubscriber_(false), alive_modelParametersSubscriber_(false),
          alive_poseSubscriber_(false)
{
  timerStarted_ = false;
  file_.open((file_name + std::string(".csv")).c_str());

  switch (mode) {
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
  subscribeToAll();

}

void AllDataMultiLogger::subscribeToAll(){

  cloudSizeSubscriber_ = nh.subscribe<std_msgs::Float32>("object/cloudSize", 1, &AllDataMultiLogger::cloudSizeCallback,
                                                         this);
  originalCloudSizeSubscriber_ = nh.subscribe<std_msgs::Float32>("object/originalCloudSize", 1, &AllDataMultiLogger::originalCloudSizeCallback,
                                                         this);
  filteredCloudSizeSubscriber_ = nh.subscribe<std_msgs::Float32>("object/filteredCloudSize", 1, &AllDataMultiLogger::filteredCloudSizeCallback,
                                                         this);
  noBackgroundcloudSizeSubscriber_ = nh.subscribe<std_msgs::Float32>("object/noBackgroundCloudSize", 1, &AllDataMultiLogger::noBackgroundCloudSizeCallback,
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
  file_ << "Id,TimeSecs,OriginalCloudSize,FilteredCloudSize,NoBackgroundCloudSize,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Radius,Height," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataMultiLogger::writeSQHeader() {
  file_ << "Id,TimeSecs,OriginalCloudSize,FilteredCloudSize,NoBackgroundCloudSize,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "e1,e2,a,b,c," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataMultiLogger::writePCAHeader() {
  writeRANSACBoxHeader();
}

void AllDataMultiLogger::writeRANSACBoxHeader() {
  file_ << "Id,TimeSecs,OriginalCloudSize,FilteredCloudSize,NoBackgroundCloudSize,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Width,Height,Depth," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

void AllDataMultiLogger::writeRANSACSphereHeader() {
  file_ << "Id,TimeSecs,OriginalCloudSize,FilteredCloudSize,NoBackgroundCloudSize,CloudSize,ProcessTime,LoadTime,FilterTime,BackgroundTime,EstimationTime,SymmetryTime,";
  file_ << "Radius," << "PoseX,PoseY,PoseZ,PoseRoll,PosePitch,PoseYaw\n";
}

bool AllDataMultiLogger::areAllSubscribersAlive(){
  return  alive_originalCloudSizeSubscriber_ && alive_filteredCloudSizeSubscriber_  && alive_noBackgroundCloudSizeSubscriber_
          && alive_cloudSizeSubscriber_ && alive_symmetrySubscriber_ && alive_estimationSubscriber_ && alive_backgroundSubscriber_
          && alive_filterSubscriber_ && alive_loadSubscriber_ && alive_processSubscriber_
          && alive_modelParametersSubscriber_ && alive_poseSubscriber_;
}

void AllDataMultiLogger::writeRow() {

  while(!areAllSubscribersAlive()){
    ros::spinOnce();
  }

  double time;
  if(!timerStarted_){
    time = 0;
    timerStarted_ = true;
    timer.resetTimer();
  }else{
    time = timer.getTotalTime();
  }
  ROS_INFO_STREAM(time);
  file_ << objectId_ << ",";
  file_ << time << ",";
  file_ << originalCloudSize_ << ",";
  file_ << filteredCloudSize_ << ",";
  file_ << noBackgroundCloudSize_ << ",";
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

  file_.flush();
}

void AllDataMultiLogger::modelParametersCallback(const std_msgs::Float32MultiArrayConstPtr& m){
  if(modelParameters_.size() == 0) {
    for (int i = 0; i < m->data.size(); i++) {
      modelParameters_.push_back(m->data[i]);
    }
  }
  else{
    for (int i = 0; i < m->data.size(); i++) {
      modelParameters_[i] = m->data[i];
    }
  }
  alive_modelParametersSubscriber_ = true;
}

void AllDataMultiLogger::cloudSizeCallback(const std_msgs::Float32ConstPtr& m){
  cloudSize_= m->data;
  alive_cloudSizeSubscriber_ = true;
}

void AllDataMultiLogger::originalCloudSizeCallback(const std_msgs::Float32ConstPtr& m){
  originalCloudSize_= m->data;
  alive_originalCloudSizeSubscriber_ = true;
}

void AllDataMultiLogger::filteredCloudSizeCallback(const std_msgs::Float32ConstPtr& m){
  filteredCloudSize_= m->data;
  alive_filteredCloudSizeSubscriber_ = true;
}

void AllDataMultiLogger::noBackgroundCloudSizeCallback(const std_msgs::Float32ConstPtr& m){
  noBackgroundCloudSize_= m->data;
  alive_noBackgroundCloudSizeSubscriber_ = true;
}

void AllDataMultiLogger::symmetryCallback(const std_msgs::Float32ConstPtr& m){
  symmetry_= m->data;
  alive_symmetrySubscriber_ = true;
}

void AllDataMultiLogger::backgroundCallback(const std_msgs::Float32ConstPtr& m){
  background_= m->data;
  alive_backgroundSubscriber_ = true;
}

void AllDataMultiLogger::estimationCallback(const std_msgs::Float32ConstPtr& m){
  estimation_= m->data;
  alive_estimationSubscriber_ = true;
}

void AllDataMultiLogger::filterCallback(const std_msgs::Float32ConstPtr& m){
  filter_= m->data;
  alive_filterSubscriber_ = true;
}

void AllDataMultiLogger::loadCallback(const std_msgs::Float32ConstPtr& m){
  load_ = m->data;
  alive_loadSubscriber_ = true;
}

void AllDataMultiLogger::processCallback(const std_msgs::Float32ConstPtr& m){
  process_ = m->data;
  alive_processSubscriber_ = true;
  writeRow();
}

void AllDataMultiLogger::poseCallback(const geometry_msgs::PoseConstPtr& m){
  pose_ = *m;
  alive_poseSubscriber_ = true;
}

AllDataMultiLogger::~AllDataMultiLogger() {
  file_.close();
}