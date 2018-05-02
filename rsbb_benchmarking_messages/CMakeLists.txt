cmake_minimum_required(VERSION 2.8.3)
project(rsbb_benchmarking_messages)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  SystemStatus.msg
  BenchmarkState.msg
  BmBoxState.msg
  RefBoxState.msg
  RecordRequest.msg
)

add_service_files(
  FILES
  InitBenchmark.srv
  TerminateBenchmark.srv
  StartBenchmark.srv
  ExecuteManualOperation.srv
  ManualOperationComplete.srv
  ExecuteGoal.srv
  GoalStarted.srv
  GoalComplete.srv
  RefereeScore.srv
  EndBenchmark.srv
  StopBenchmark.srv
  StopRecordRequest.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

