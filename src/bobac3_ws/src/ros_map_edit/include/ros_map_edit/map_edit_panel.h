#ifndef MAP_EDIT_PANEL_H
#define MAP_EDIT_PANEL_H

#include <rviz/panel.h>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QComboBox>
#include <QGroupBox>
#include <QMessageBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QTabWidget>
#include <QProgressBar>
#include <QListWidget>
#include <QCheckBox>
#include <QInputDialog>
#include <QProcess>
#include <QProgressDialog>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

namespace ros_map_edit
{

class MapFileManager;

class MapEditPanel : public rviz::Panel
{
Q_OBJECT
public:
  MapEditPanel(QWidget* parent = 0);
  virtual ~MapEditPanel();

  virtual void onInitialize();

private Q_SLOTS:
  void saveAllFiles();
  void saveToRemote();
  void openMap();
  void openLocalMap();
  void openRemoteMap();
  void testSSHConnection();
  void setPassword();
  void refreshRemoteFileList();
  void onRemoteFileSelected();
  void downloadAndOpenRemoteMap();
  void handleScpFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void handleScpError(QProcess::ProcessError error);
  void clearVirtualWalls();
  void clearRegions();

private:
  void setupUI();
  void setupLocalTab();
  void setupRemoteTab();
  void loadAndPublishMap(const std::string& filename);
  void publishMap(const nav_msgs::OccupancyGrid& map);
  void loadCorrespondingFiles(const std::string& map_file_path);
  void clearAllMessages();
  std::string getCurrentMapFile();
  
  // SCP相关功能
  bool transferFileViaSCP(const QString& source, const QString& target, bool is_upload);
  QString generateTempFilePath() const;
  void updateStatus(const QString& message, bool is_error = false);
  void saveConnectionSettings();
  void loadConnectionSettings();
  
  // 地图文件集下载相关
  void downloadMapFileSequence(const QString& temp_dir, const QString& remote_host, 
                              const QString& remote_user, const QString& remote_dir,
                              const QString& yaml_filename, const QString& pgm_filename,
                              const QString& json_filename, const QString& region_filename,
                              int step);
  void onAllFilesDownloaded(const QString& temp_dir, const QString& yaml_filename);
  
  // 清除后自动保存相关
  void saveVirtualWallsFileAfterClear();
  void saveRegionsFileAfterClear();

  // UI Components
  QVBoxLayout* main_layout_;
  
  // 一键保存组
  QGroupBox* save_group_;
  QPushButton* save_all_btn_;
  QPushButton* save_to_remote_btn_;
  QPushButton* clear_virtual_walls_btn_;
  QPushButton* clear_regions_btn_;
  QPushButton* open_map_btn_;
  QLabel* current_map_label_;
  
  // 地图打开选择
  QTabWidget* map_source_tabs_;
  
  // 本地地图选项卡
  QWidget* local_tab_;
  QPushButton* open_local_btn_;
  
  // 远程地图选项卡 (SSH/SCP)
  QWidget* remote_tab_;
  QLineEdit* ssh_host_edit_;
  QLineEdit* ssh_port_edit_;
  QLineEdit* ssh_username_edit_;
  QLineEdit* ssh_remote_dir_edit_;
  QPushButton* password_btn_;
  QPushButton* test_connection_btn_;
  QPushButton* refresh_files_btn_;
  QListWidget* remote_files_list_;
  QPushButton* download_open_btn_;
  QProgressBar* download_progress_;
  QLabel* connection_status_label_;
  QGroupBox* remote_config_group_;
  QProgressDialog* progress_dialog_;
  
  // 状态显示
  QLabel* status_label_;
  QLabel* info_label_;
  
  // Map file manager
  MapFileManager* file_manager_;
  
  // 当前地图文件路径和状态
  QString current_map_file_;
  QString current_password_;
  QString current_temp_file_;
  bool is_remote_map_;
  QString remote_map_base_name_;  // 存储远程地图的原始基础名称（不含路径和扩展名）
  
  // SCP传输相关
  QProcess* scp_process_;
  bool scp_password_sent_;
  QString scp_output_;
  
  // 测试连接相关
  bool test_password_sent_;
  QString test_output_;
};

} // end namespace ros_map_edit

#endif // MAP_EDIT_PANEL_H 