#ifndef MULTI_LIDAR_SLAM_OPTIMIZED__MAP_SAVE_PANEL_HPP_
#define MULTI_LIDAR_SLAM_OPTIMIZED__MAP_SAVE_PANEL_HPP_

#include <memory>
#include <string>

#include <QProcess>
#include <QString>

#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/write_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

class QLabel;
class QLineEdit;
class QPushButton;

namespace control_center_slam
{

class MapSavePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MapSavePanel(QWidget * parent = nullptr);
  ~MapSavePanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void saveCheckpoint();
  void optimizeAndSaveFinal();
  void mapSaverFinished(int exit_code, QProcess::ExitStatus exit_status);

private:
  using FinishTrajectory = cartographer_ros_msgs::srv::FinishTrajectory;
  using WriteState = cartographer_ros_msgs::srv::WriteState;

  bool prepareMapName();
  void beginSave(bool finish_before_save);
  void finishTrajectory();
  void writeState();
  void startMapSaver();
  void setStatus(const QString & text, bool error = false);
  void fail(const QString & text);
  void setBusy(bool busy);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<FinishTrajectory>::SharedPtr finish_client_;
  rclcpp::Client<WriteState>::SharedPtr write_state_client_;

  QLineEdit * map_name_edit_;
  QPushButton * checkpoint_button_;
  QPushButton * final_save_button_;
  QLabel * status_label_;
  QLabel * directory_label_;
  QProcess * map_saver_process_;

  QString maps_directory_;
  QString map_name_;
  QString map_prefix_;
  bool busy_;
  bool trajectory_finished_;
  bool finish_before_save_;
};

}  // namespace control_center_slam

#endif  // MULTI_LIDAR_SLAM_OPTIMIZED__MAP_SAVE_PANEL_HPP_
