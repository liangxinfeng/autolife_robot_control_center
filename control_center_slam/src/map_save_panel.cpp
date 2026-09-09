#include "control_center_slam/map_save_panel.hpp"

#include <chrono>
#include <filesystem>
#include <functional>
#include <utility>

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QRegularExpression>
#include <QVBoxLayout>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace control_center_slam
{

MapSavePanel::MapSavePanel(QWidget * parent)
: rviz_common::Panel(parent),
  map_name_edit_(new QLineEdit(this)),
  checkpoint_button_(new QPushButton(tr("保存当前地图（继续建图）"), this)),
  final_save_button_(new QPushButton(tr("最终优化并保存（结束建图）"), this)),
  status_label_(new QLabel(tr("等待 SLAM 服务"), this)),
  directory_label_(new QLabel(this)),
  map_saver_process_(nullptr),
  busy_(false),
  trajectory_finished_(false),
  finish_before_save_(false)
{
  auto * title = new QLabel(tr("Cartographer 地图保存"), this);
  QFont title_font = title->font();
  title_font.setPointSize(title_font.pointSize() + 2);
  title_font.setBold(true);
  title->setFont(title_font);

  auto * warning = new QLabel(
    tr(
      "普通保存不会中断建图；只有“最终优化并保存”会结束当前轨迹，"
      "结束后本轮建图不能继续。"), this);
  warning->setWordWrap(true);
  warning->setStyleSheet("color: #d97706;");

  auto * name_layout = new QHBoxLayout();
  name_layout->addWidget(new QLabel(tr("地图名称："), this));
  name_layout->addWidget(map_name_edit_, 1);

  auto * button_layout = new QVBoxLayout();
  button_layout->addWidget(checkpoint_button_);
  button_layout->addWidget(final_save_button_);

  map_name_edit_->setText(
    QDateTime::currentDateTime().toString("map_yyyyMMdd_HHmmss"));
  map_name_edit_->setPlaceholderText(tr("例如：二楼完整地图"));

  directory_label_->setWordWrap(true);
  directory_label_->setStyleSheet("color: #6b7280;");
  status_label_->setWordWrap(true);

  auto * layout = new QVBoxLayout();
  layout->addWidget(title);
  layout->addWidget(warning);
  layout->addLayout(name_layout);
  layout->addLayout(button_layout);
  layout->addWidget(status_label_);
  layout->addWidget(directory_label_);
  layout->addStretch(1);
  setLayout(layout);

  connect(
    checkpoint_button_, &QPushButton::clicked,
    this, &MapSavePanel::saveCheckpoint);
  connect(
    final_save_button_, &QPushButton::clicked,
    this, &MapSavePanel::optimizeAndSaveFinal);
}

MapSavePanel::~MapSavePanel()
{
  if (map_saver_process_ && map_saver_process_->state() != QProcess::NotRunning) {
    map_saver_process_->terminate();
    map_saver_process_->waitForFinished(1000);
  }
}

void MapSavePanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    fail(tr("RViz ROS 节点不可用"));
    return;
  }

  node_ = abstraction->get_raw_node();
  finish_client_ = node_->create_client<FinishTrajectory>("/finish_trajectory");
  write_state_client_ = node_->create_client<WriteState>("/write_state");

  try {
    const std::filesystem::path share_directory =
      ament_index_cpp::get_package_share_directory("control_center_slam");
    const auto install_prefix = share_directory.parent_path().parent_path();
    const auto workspace_directory =
      install_prefix.parent_path().parent_path();
    auto source_package_directory =
      workspace_directory / "src" / "control_center_slam";
    const auto grouped_source_package_directory =
      workspace_directory / "src" / "autolife_robot_control_center" /
      "control_center_slam";
    if (!std::filesystem::exists(source_package_directory / "package.xml") &&
      std::filesystem::exists(grouped_source_package_directory / "package.xml"))
    {
      source_package_directory = grouped_source_package_directory;
    }
    const auto source_maps_directory = source_package_directory / "maps";
    const auto installed_maps_directory = share_directory / "maps";

    std::filesystem::path selected_maps_directory = installed_maps_directory;
    if (std::filesystem::exists(source_package_directory / "package.xml")) {
      selected_maps_directory = source_maps_directory;
    }
    std::filesystem::create_directories(selected_maps_directory);
    maps_directory_ = QString::fromStdString(
      std::filesystem::weakly_canonical(selected_maps_directory).string());
  } catch (const std::exception & exception) {
    fail(tr("无法定位功能包 maps 目录：%1").arg(exception.what()));
    return;
  }

  directory_label_->setText(tr("保存目录：%1").arg(maps_directory_));
  RCLCPP_INFO(
    node_->get_logger(), "Map save panel directory: %s",
    maps_directory_.toStdString().c_str());
  setStatus(tr("已连接；输入名称后点击按钮"));
}

bool MapSavePanel::prepareMapName()
{
  QString name = map_name_edit_->text().trimmed();
  const QStringList suffixes = {".pbstream", ".yaml", ".pgm"};
  for (const auto & suffix : suffixes) {
    if (name.endsWith(suffix, Qt::CaseInsensitive)) {
      name.chop(suffix.size());
      break;
    }
  }

  const QRegularExpression valid_name(
    "^[\\p{L}\\p{N}_-]+$",
    QRegularExpression::UseUnicodePropertiesOption);
  if (name.isEmpty()) {
    fail(tr("地图名称不能为空"));
    return false;
  }
  if (!valid_name.match(name).hasMatch()) {
    fail(tr("名称只能包含中文、字母、数字、下划线和连字符"));
    return false;
  }

  map_name_ = name;
  map_prefix_ = QDir(maps_directory_).filePath(map_name_);
  map_name_edit_->setText(map_name_);
  return true;
}

void MapSavePanel::saveCheckpoint()
{
  beginSave(false);
}

void MapSavePanel::optimizeAndSaveFinal()
{
  beginSave(true);
}

void MapSavePanel::beginSave(bool finish_before_save)
{
  if (busy_ || !node_ || !prepareMapName()) {
    return;
  }

  finish_before_save_ = finish_before_save;
  setBusy(true);
  if (!finish_before_save_ || trajectory_finished_) {
    writeState();
  } else {
    finishTrajectory();
  }
}

void MapSavePanel::finishTrajectory()
{
  if (!finish_client_->wait_for_service(std::chrono::seconds(0))) {
    fail(tr("找不到 /finish_trajectory，请确认 Cartographer 正在运行"));
    return;
  }

  setStatus(tr("1/3 正在结束轨迹并执行最终优化…"));
  auto request = std::make_shared<FinishTrajectory::Request>();
  request->trajectory_id = 0;
  finish_client_->async_send_request(
    request,
    [this](rclcpp::Client<FinishTrajectory>::SharedFuture future) {
      try {
        const auto response = future.get();
        QMetaObject::invokeMethod(
          this,
          [this, response]() {
            if (response->status.code != 0) {
              fail(
                tr("最终优化失败：%1")
                .arg(QString::fromStdString(response->status.message)));
              return;
            }
            trajectory_finished_ = true;
            writeState();
          },
          Qt::QueuedConnection);
      } catch (const std::exception & exception) {
        QMetaObject::invokeMethod(
          this,
          [this, message = QString::fromUtf8(exception.what())]() {
            fail(tr("调用最终优化服务异常：%1").arg(message));
          },
          Qt::QueuedConnection);
      }
    });
}

void MapSavePanel::writeState()
{
  if (!write_state_client_->wait_for_service(std::chrono::seconds(0))) {
    fail(tr("找不到 /write_state 服务"));
    return;
  }

  setStatus(
    finish_before_save_ ?
    tr("2/3 正在保存 Cartographer 最终状态…") :
    tr("1/2 正在保存可继续建图的 Cartographer 快照…"));
  auto request = std::make_shared<WriteState::Request>();
  request->filename = (map_prefix_ + ".pbstream").toStdString();
  // FinishTrajectory does not seal the final active submap grids.
  request->include_unfinished_submaps = true;
  write_state_client_->async_send_request(
    request,
    [this](rclcpp::Client<WriteState>::SharedFuture future) {
      try {
        const auto response = future.get();
        QMetaObject::invokeMethod(
          this,
          [this, response]() {
            if (response->status.code != 0) {
              fail(
                tr("保存 pbstream 失败：%1")
                .arg(QString::fromStdString(response->status.message)));
              return;
            }
            startMapSaver();
          },
          Qt::QueuedConnection);
      } catch (const std::exception & exception) {
        QMetaObject::invokeMethod(
          this,
          [this, message = QString::fromUtf8(exception.what())]() {
            fail(tr("调用状态保存服务异常：%1").arg(message));
          },
          Qt::QueuedConnection);
      }
    });
}

void MapSavePanel::startMapSaver()
{
  setStatus(
    finish_before_save_ ?
    tr("3/3 正在生成最终 YAML/PGM 导航地图…") :
    tr("2/2 正在生成当前 YAML/PGM 导航地图…"));

  if (map_saver_process_) {
    map_saver_process_->deleteLater();
  }
  map_saver_process_ = new QProcess(this);
  map_saver_process_->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  connect(
    map_saver_process_,
    qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
    this,
    &MapSavePanel::mapSaverFinished);
  connect(
    map_saver_process_,
    &QProcess::errorOccurred,
    this,
    [this](QProcess::ProcessError error) {
      if (error == QProcess::FailedToStart) {
        fail(tr("无法启动 nav2 map_saver_cli"));
      }
    });

  const QString executable = "/opt/ros/jazzy/lib/nav2_map_server/map_saver_cli";
  const QStringList arguments = {
    "-f", map_prefix_,
    "--ros-args", "-p", "map_subscribe_transient_local:=true"
  };
  map_saver_process_->start(executable, arguments);
}

void MapSavePanel::mapSaverFinished(
  int exit_code, QProcess::ExitStatus exit_status)
{
  if (!busy_) {
    return;
  }
  if (exit_status != QProcess::NormalExit || exit_code != 0) {
    const QString output =
      QString::fromUtf8(map_saver_process_->readAllStandardError()).trimmed();
    fail(
      tr("生成 YAML/PGM 失败（exit=%1）：%2")
      .arg(exit_code)
      .arg(output));
    return;
  }

  const bool pbstream_ok = QFileInfo::exists(map_prefix_ + ".pbstream");
  const bool yaml_ok = QFileInfo::exists(map_prefix_ + ".yaml");
  const bool pgm_ok = QFileInfo::exists(map_prefix_ + ".pgm");
  if (!pbstream_ok || !yaml_ok || !pgm_ok) {
    fail(tr("保存程序已退出，但地图文件不完整，请检查 maps 目录"));
    return;
  }

  setBusy(false);
  if (finish_before_save_) {
    final_save_button_->setText(tr("再次保存最终地图"));
    setStatus(
      tr("最终优化与保存完成：%1.{pbstream,yaml,pgm}").arg(map_prefix_));
  } else {
    setStatus(
      tr("快照保存完成，建图仍在继续：%1.{pbstream,yaml,pgm}").arg(map_prefix_));
  }
}

void MapSavePanel::setStatus(const QString & text, bool error)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(error ? "color: #dc2626;" : "color: #15803d;");
}

void MapSavePanel::fail(const QString & text)
{
  setBusy(false);
  setStatus(text, true);
}

void MapSavePanel::setBusy(bool busy)
{
  busy_ = busy;
  checkpoint_button_->setEnabled(!busy);
  final_save_button_->setEnabled(!busy);
  map_name_edit_->setEnabled(!busy);
}

}  // namespace control_center_slam

PLUGINLIB_EXPORT_CLASS(
  control_center_slam::MapSavePanel,
  rviz_common::Panel)
