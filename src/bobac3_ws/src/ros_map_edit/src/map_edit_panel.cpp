#include "ros_map_edit/map_edit_panel.h"
#include "ros_map_edit/map_file_manager.h"
#include "ros_map_edit/tool_manager.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>
#include <QTabWidget>
#include <QRadioButton>
#include <QButtonGroup>
#include <QProgressBar>
#include <QListWidget>
#include <QThread>
#include <QApplication>
#include <visualization_msgs/MarkerArray.h>
#include <QDateTime>
#include <QSettings>
#include <QFormLayout>
#include <QCheckBox>
#include <QInputDialog>
#include <QProcess>
#include <QTextStream>

namespace ros_map_edit
{

MapEditPanel::MapEditPanel(QWidget* parent)
  : rviz::Panel(parent)
  , main_layout_(nullptr)
  , file_manager_(nullptr)
  , scp_process_(nullptr)
  , progress_dialog_(nullptr)
  , is_remote_map_(false)
{
  setupUI();
  file_manager_ = new MapFileManager();
  
  // 加载保存的连接设置
  loadConnectionSettings();
}

MapEditPanel::~MapEditPanel()
{
  // 保存当前设置
  saveConnectionSettings();
  
  // 清理临时文件/目录
  if (!current_temp_file_.isEmpty()) {
    QFileInfo fileInfo(current_temp_file_);
    if (fileInfo.isDir()) {
      // 如果是目录，递归删除
      QDir(current_temp_file_).removeRecursively();
    } else {
      // 如果是文件，直接删除
      QFile::remove(current_temp_file_);
    }
  }
  
  // 清理SCP进程
  if(scp_process_) {
    if(scp_process_->state() == QProcess::Running) {
      scp_process_->terminate();
      scp_process_->waitForFinished();
    }
    delete scp_process_;
  }
  // 释放其他资源
  delete file_manager_;
  delete progress_dialog_;
}

void MapEditPanel::onInitialize()
{
  updateStatus("就绪 - 请先打开地图文件");
}

void MapEditPanel::setupUI()
{
  main_layout_ = new QVBoxLayout;
  
  // 一键保存组
  save_group_ = new QGroupBox("文件管理");
  QVBoxLayout* save_layout = new QVBoxLayout();
  
  // 当前地图显示
  current_map_label_ = new QLabel("当前地图: 未加载");
  current_map_label_->setStyleSheet("QLabel { color: #333; font-weight: bold; padding: 5px; }");
  save_layout->addWidget(current_map_label_);
  
  // 保存按钮组
  QHBoxLayout* save_btn_layout = new QHBoxLayout();
  
  save_all_btn_ = new QPushButton("保存到本地");
  save_all_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-size: 12px; padding: 6px; }");
  save_btn_layout->addWidget(save_all_btn_);
  
  save_to_remote_btn_ = new QPushButton("保存到远程");
  save_to_remote_btn_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-size: 12px; padding: 6px; }");
  save_to_remote_btn_->setEnabled(false);
  save_btn_layout->addWidget(save_to_remote_btn_);
  
  save_layout->addLayout(save_btn_layout);
  
  // 清除操作按钮组
  QHBoxLayout* clear_btn_layout = new QHBoxLayout();
  
  clear_virtual_walls_btn_ = new QPushButton("清除虚拟墙");
  clear_virtual_walls_btn_->setStyleSheet("QPushButton { background-color: #F44336; color: white; font-size: 12px; padding: 6px; }");
  clear_btn_layout->addWidget(clear_virtual_walls_btn_);
  
  clear_regions_btn_ = new QPushButton("清除区域");
  clear_regions_btn_->setStyleSheet("QPushButton { background-color: #E91E63; color: white; font-size: 12px; padding: 6px; }");
  clear_btn_layout->addWidget(clear_regions_btn_);
  
  save_layout->addLayout(clear_btn_layout);
  
  // 创建选项卡控件用于选择地图来源
  map_source_tabs_ = new QTabWidget();
  setupLocalTab();
  setupRemoteTab();
  
  save_layout->addWidget(map_source_tabs_);
  
  save_group_->setLayout(save_layout);
  
  // 状态显示
  status_label_ = new QLabel("就绪 - 请先打开一个地图文件");
  status_label_->setStyleSheet("QLabel { background-color: #f0f0f0; padding: 8px; border: 1px solid #ccc; border-radius: 4px; }");
  
  // 文件说明
  info_label_ = new QLabel(
    "保存文件说明:\n"
    "• map.yaml - 地图配置文件\n"
    "• map.pgm - 地图图像文件\n" 
    "• map.json - 虚拟墙数据\n"
    "• map_region.json - 区域数据\n\n"
    "提示: 文件将保存到当前地图的同一目录");
  info_label_->setStyleSheet("QLabel { color: #666; font-size: 11px; padding: 8px; }");
  
  // 进度对话框设置
  progress_dialog_ = new QProgressDialog(this);
  progress_dialog_->setWindowModality(Qt::WindowModal);
  progress_dialog_->setCancelButton(nullptr); // 暂时禁用取消按钮
  progress_dialog_->reset();
  progress_dialog_->setMinimumDuration(0);
  
  // 组装主布局
  main_layout_->addWidget(save_group_);
  main_layout_->addWidget(new QLabel("状态:"));
  main_layout_->addWidget(status_label_);
  main_layout_->addWidget(info_label_);
  main_layout_->addStretch();
  
  setLayout(main_layout_);
  
  // 连接信号
  connect(save_all_btn_, SIGNAL(clicked()), this, SLOT(saveAllFiles()));
  connect(save_to_remote_btn_, SIGNAL(clicked()), this, SLOT(saveToRemote()));
  connect(clear_virtual_walls_btn_, SIGNAL(clicked()), this, SLOT(clearVirtualWalls()));
  connect(clear_regions_btn_, SIGNAL(clicked()), this, SLOT(clearRegions()));
}

void MapEditPanel::setupLocalTab()
{
  local_tab_ = new QWidget();
  QVBoxLayout* local_layout = new QVBoxLayout();
  
  // 本地地图打开按钮
  open_local_btn_ = new QPushButton("选择本地地图文件");
  open_local_btn_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-size: 12px; padding: 6px; }");
  local_layout->addWidget(open_local_btn_);
  
  // 添加说明
  QLabel* local_info = new QLabel("支持格式: YAML配置文件 (*.yaml, *.yml)\n或PGM图像文件 (*.pgm)");
  local_info->setStyleSheet("QLabel { color: #666; font-size: 10px; padding: 4px; }");
  local_layout->addWidget(local_info);
  
  local_layout->addStretch();
  local_tab_->setLayout(local_layout);
  map_source_tabs_->addTab(local_tab_, "本地地图");
  
  // 连接信号
  connect(open_local_btn_, SIGNAL(clicked()), this, SLOT(openLocalMap()));
}

void MapEditPanel::setupRemoteTab()
{
  remote_tab_ = new QWidget();
  QVBoxLayout* remote_layout = new QVBoxLayout();
  
  // SSH配置组
  remote_config_group_ = new QGroupBox("SSH服务器配置");
  QFormLayout* ssh_config_layout = new QFormLayout();
  
  // 服务器地址
  ssh_host_edit_ = new QLineEdit("example.local");
  ssh_host_edit_->setPlaceholderText("例如: example.local 或 server.example.com");
  ssh_config_layout->addRow("服务器地址:", ssh_host_edit_);
  
  // 端口
  ssh_port_edit_ = new QLineEdit("22");
  ssh_port_edit_->setMaximumWidth(80);
  ssh_config_layout->addRow("SSH端口:", ssh_port_edit_);
  
  // 用户名
  ssh_username_edit_ = new QLineEdit("oryxbot");
  ssh_username_edit_->setPlaceholderText("SSH用户名");
  ssh_config_layout->addRow("用户名:", ssh_username_edit_);
  
  // 远程目录
  ssh_remote_dir_edit_ = new QLineEdit("/home/oryxbot/.reinovo/maps");
  ssh_remote_dir_edit_->setPlaceholderText("例如: /home/user/maps");
  ssh_config_layout->addRow("远程目录:", ssh_remote_dir_edit_);
  
  remote_config_group_->setLayout(ssh_config_layout);
  remote_layout->addWidget(remote_config_group_);
  
  // 密码设置按钮
  password_btn_ = new QPushButton("设置SSH密码");
  password_btn_->setStyleSheet("QPushButton { background-color: #607D8B; color: white; }");
  remote_layout->addWidget(password_btn_);
  
  // 操作按钮
  QHBoxLayout* btn_layout = new QHBoxLayout();
  test_connection_btn_ = new QPushButton("测试SSH连接");
  test_connection_btn_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; }");
  test_connection_btn_->setEnabled(false); // 默认禁用
  btn_layout->addWidget(test_connection_btn_);
  
  refresh_files_btn_ = new QPushButton("刷新文件列表");
  refresh_files_btn_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; }");
  refresh_files_btn_->setEnabled(false);
  btn_layout->addWidget(refresh_files_btn_);
  
  remote_layout->addLayout(btn_layout);
  
  // 连接状态
  connection_status_label_ = new QLabel("未连接");
  connection_status_label_->setStyleSheet("QLabel { color: #f44336; font-weight: bold; padding: 2px; }");
  remote_layout->addWidget(connection_status_label_);
  
  // 远程文件列表
  QLabel* files_label = new QLabel("远程地图文件:");
  remote_layout->addWidget(files_label);
  
  remote_files_list_ = new QListWidget();
  remote_files_list_->setMaximumHeight(120);
  remote_layout->addWidget(remote_files_list_);
  
  // 下载和打开按钮
  download_open_btn_ = new QPushButton("下载并打开所选地图");
  download_open_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-size: 12px; padding: 6px; }");
  download_open_btn_->setEnabled(false);
  remote_layout->addWidget(download_open_btn_);
  
  // 下载进度条
  download_progress_ = new QProgressBar();
  download_progress_->setVisible(false);
  remote_layout->addWidget(download_progress_);
  
  remote_tab_->setLayout(remote_layout);
  map_source_tabs_->addTab(remote_tab_, "远程地图 (SSH)");
  
  // 连接信号
  connect(password_btn_, &QPushButton::clicked, this, &MapEditPanel::setPassword);
  connect(test_connection_btn_, &QPushButton::clicked, this, &MapEditPanel::testSSHConnection);
  connect(refresh_files_btn_, &QPushButton::clicked, this, &MapEditPanel::refreshRemoteFileList);
  connect(remote_files_list_, &QListWidget::itemSelectionChanged, this, &MapEditPanel::onRemoteFileSelected);
  connect(download_open_btn_, &QPushButton::clicked, this, &MapEditPanel::downloadAndOpenRemoteMap);
  
  // SSH配置变化时更新按钮状态
  connect(ssh_host_edit_, &QLineEdit::textChanged, [this](){
    bool has_config = !ssh_host_edit_->text().isEmpty() && 
                     !ssh_username_edit_->text().isEmpty() &&
                     !ssh_remote_dir_edit_->text().isEmpty();
    test_connection_btn_->setEnabled(has_config && !current_password_.isEmpty());
    save_to_remote_btn_->setEnabled(has_config && !current_password_.isEmpty());
  });
  connect(ssh_username_edit_, &QLineEdit::textChanged, [this](){
    bool has_config = !ssh_host_edit_->text().isEmpty() && 
                     !ssh_username_edit_->text().isEmpty() &&
                     !ssh_remote_dir_edit_->text().isEmpty();
    test_connection_btn_->setEnabled(has_config && !current_password_.isEmpty());
    save_to_remote_btn_->setEnabled(has_config && !current_password_.isEmpty());
  });
  connect(ssh_remote_dir_edit_, &QLineEdit::textChanged, [this](){
    bool has_config = !ssh_host_edit_->text().isEmpty() && 
                     !ssh_username_edit_->text().isEmpty() &&
                     !ssh_remote_dir_edit_->text().isEmpty();
    test_connection_btn_->setEnabled(has_config && !current_password_.isEmpty());
    save_to_remote_btn_->setEnabled(has_config && !current_password_.isEmpty());
  });
}

void MapEditPanel::saveAllFiles()
{
  // 获取当前地图文件路径
  std::string current_map_file = getCurrentMapFile();
  if (current_map_file.empty())
  {
    QMessageBox::warning(this, "警告", "请先加载一个地图文件作为基准");
    return;
  }
  
  // 从当前地图文件路径提取目录和基础名称
  QFileInfo map_info(QString::fromStdString(current_map_file));
  QString base_dir = map_info.absolutePath();
  QString base_name = map_info.baseName(); // 不包含扩展名
  
  // 构建保存文件路径
  QString yaml_file = base_dir + "/" + base_name + ".yaml";
  QString pgm_file = base_dir + "/" + base_name + ".pgm";
  QString vw_file = base_dir + "/" + base_name + ".json";
  QString region_file = base_dir + "/" + base_name + "_region.json";
  
  bool success = true;
  QStringList saved_files;
  QStringList failed_files;
  
  try
  {
    // 获取工具管理器实例
    ToolManager& toolManager = ToolManager::getInstance();
    
    // 1. 保存地图 (yaml + pgm)
    MapEraserTool* eraserTool = toolManager.getMapEraserTool();
    if (eraserTool && eraserTool->getCurrentMap().data.size() > 0)
    {
      // 使用橡皮擦工具的当前地图数据
      if (file_manager_->saveMapFiles(yaml_file.toStdString(), eraserTool->getCurrentMap()))
      {
        saved_files << "地图文件 (yaml+pgm)";
      }
      else
      {
        failed_files << "地图文件: " + QString::fromStdString(file_manager_->getLastError());
        success = false;
      }
    }
    else
    {
      // 使用默认方法（从话题获取）
      if (file_manager_->saveMapFiles(yaml_file.toStdString()))
      {
        saved_files << "地图文件 (yaml+pgm)";
      }
      else
      {
        failed_files << "地图文件: " + QString::fromStdString(file_manager_->getLastError());
        success = false;
      }
    }
    
    // 2. 保存虚拟墙
    VirtualWallTool* wallTool = toolManager.getVirtualWallTool();
    if (wallTool)
    {
      std::vector<VirtualWall> walls = wallTool->getVirtualWalls();
      if (file_manager_->saveVirtualWallsFile(vw_file.toStdString(), walls))
      {
        saved_files << "虚拟墙文件 (" + QString::number(walls.size()) + " 个墙体)";
      }
      else
      {
        failed_files << "虚拟墙文件: " + QString::fromStdString(file_manager_->getLastError());
        success = false;
      }
    }
    else
    {
      // 使用默认方法
      if (file_manager_->saveVirtualWallsFile(vw_file.toStdString()))
      {
        saved_files << "虚拟墙文件";
      }
      else
      {
        failed_files << "虚拟墙文件: " + QString::fromStdString(file_manager_->getLastError());
        success = false;
      }
    }
    
    // 3. 保存区域
    RegionTool* regionTool = toolManager.getRegionTool();
    if (regionTool)
    {
      std::vector<Region> regions = regionTool->getRegions();
      if (file_manager_->saveRegionsFile(region_file.toStdString(), regions))
      {
        saved_files << "区域文件 (" + QString::number(regions.size()) + " 个区域)";
      }
      else
      {
        failed_files << "区域文件: " + QString::fromStdString(file_manager_->getLastError());
        success = false;
      }
    }
    else
    {
      // 使用默认方法
      if (file_manager_->saveRegionsFile(region_file.toStdString()))
      {
        saved_files << "区域文件";
      }
      else
      {
        failed_files << "区域文件: " + QString::fromStdString(file_manager_->getLastError());
        success = false;
      }
    }
    
    // 显示结果
    QString message = "保存到: " + base_dir + "\n\n";
    if (!saved_files.isEmpty())
    {
      message += "✓ 成功保存: " + saved_files.join(", ") + "\n";
    }
    if (!failed_files.isEmpty())
    {
      message += "✗ 保存失败: " + failed_files.join(", ") + "\n";
    }
    
    if (success)
    {
      status_label_->setText("所有文件已保存: " + base_name);
      QMessageBox::information(this, "保存成功", message);
    }
    else
    {
      status_label_->setText("部分文件保存失败");
      QMessageBox::warning(this, "保存部分失败", message);
    }
  }
  catch (const std::exception& e)
  {
    QString error_msg = "保存过程中出现错误: " + QString::fromStdString(e.what());
    status_label_->setText("保存失败");
    QMessageBox::critical(this, "保存错误", error_msg);
  }
}

void MapEditPanel::saveToRemote()
{
    try
    {
        // 检查SSH连接状态
        if (connection_status_label_->text() != "连接成功!")
        {
            QMessageBox::warning(this, "SSH未连接", "请先测试SSH连接并确保连接成功");
            return;
        }
        
        // 获取当前地图文件路径
        std::string current_map_file = getCurrentMapFile();
        if (current_map_file.empty())
        {
            QMessageBox::warning(this, "警告", "请先加载一个地图文件");
            return;
        }
        
        // 获取文件名作为远程基础名称
        QFileInfo map_info(QString::fromStdString(current_map_file));
        QString base_name = map_info.baseName(); // 不包含扩展名
        
        // 让用户输入远程文件名
        bool ok;
        QString remote_name = QInputDialog::getText(this, "保存到远程",
                                                   "输入远程文件名 (不包含扩展名):",
                                                   QLineEdit::Normal,
                                                   base_name, &ok);
        
        if (!ok || remote_name.isEmpty())
        {
            return;
        }
        
        // 获取SSH配置
        QString remote_host = ssh_host_edit_->text().trimmed();
        QString remote_user = ssh_username_edit_->text().trimmed();
        QString remote_dir = ssh_remote_dir_edit_->text().trimmed();
        
        updateStatus("正在准备上传文件到远程服务器...");
        
        // 先在本地生成临时文件
        QString temp_dir = QDir::tempPath() + "/ros_map_upload_" + 
                          QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
        QDir().mkpath(temp_dir);
        
        QString yaml_file = temp_dir + "/" + remote_name + ".yaml";
        QString pgm_file = temp_dir + "/" + remote_name + ".pgm";
        QString vw_file = temp_dir + "/" + remote_name + ".json";
        QString region_file = temp_dir + "/" + remote_name + "_region.json";
        
        bool success = true;
        QStringList saved_files;
        QStringList failed_files;
        
        // 获取工具管理器实例
        ToolManager& toolManager = ToolManager::getInstance();
        
        // 1. 保存地图 (yaml + pgm) 到临时目录
        MapEraserTool* eraserTool = toolManager.getMapEraserTool();
        if (eraserTool && eraserTool->getCurrentMap().data.size() > 0)
        {
            // 使用橡皮擦工具的当前地图数据
            if (file_manager_->saveMapFiles(yaml_file.toStdString(), eraserTool->getCurrentMap()))
            {
                saved_files << "地图文件 (yaml+pgm)";
            }
            else
            {
                failed_files << "地图文件: " + QString::fromStdString(file_manager_->getLastError());
                success = false;
            }
        }
        else
        {
            // 使用默认方法（从话题获取）
            if (file_manager_->saveMapFiles(yaml_file.toStdString()))
            {
                saved_files << "地图文件 (yaml+pgm)";
            }
            else
            {
                failed_files << "地图文件: " + QString::fromStdString(file_manager_->getLastError());
                success = false;
            }
        }
        
        // 2. 保存虚拟墙到临时目录
        VirtualWallTool* wallTool = toolManager.getVirtualWallTool();
        if (wallTool)
        {
            std::vector<VirtualWall> walls = wallTool->getVirtualWalls();
            if (!walls.empty())
            {
                if (file_manager_->saveVirtualWallsFile(vw_file.toStdString(), walls))
                {
                    saved_files << "虚拟墙文件 (" + QString::number(walls.size()) + " 个墙体)";
                }
                else
                {
                    failed_files << "虚拟墙文件: " + QString::fromStdString(file_manager_->getLastError());
                    success = false;
                }
            }
        }
        
        // 3. 保存区域到临时目录
        RegionTool* regionTool = toolManager.getRegionTool();
        if (regionTool)
        {
            std::vector<Region> regions = regionTool->getRegions();
            if (!regions.empty())
            {
                if (file_manager_->saveRegionsFile(region_file.toStdString(), regions))
                {
                    saved_files << "区域文件 (" + QString::number(regions.size()) + " 个区域)";
                }
                else
                {
                    failed_files << "区域文件: " + QString::fromStdString(file_manager_->getLastError());
                    success = false;
                }
            }
        }
        
        // 如果本地保存成功，则上传到远程
        if (success)
        {
            updateStatus("正在上传文件到远程服务器...");
            
            // 构建远程目标路径
            QString remote_yaml = QString("%1@%2:%3/%4.yaml").arg(remote_user).arg(remote_host).arg(remote_dir).arg(remote_name);
            QString remote_pgm = QString("%1@%2:%3/%4.pgm").arg(remote_user).arg(remote_host).arg(remote_dir).arg(remote_name);
            QString remote_vw = QString("%1@%2:%3/%4.json").arg(remote_user).arg(remote_host).arg(remote_dir).arg(remote_name);
            QString remote_region = QString("%1@%2:%3/%4_region.json").arg(remote_user).arg(remote_host).arg(remote_dir).arg(remote_name);
            
            QStringList upload_files;
            QStringList upload_targets;
            
            // 准备上传文件列表
            if (QFile::exists(yaml_file)) {
                upload_files << yaml_file;
                upload_targets << remote_yaml;
            }
            if (QFile::exists(pgm_file)) {
                upload_files << pgm_file;
                upload_targets << remote_pgm;
            }
            if (QFile::exists(vw_file)) {
                upload_files << vw_file;
                upload_targets << remote_vw;
            }
            if (QFile::exists(region_file)) {
                upload_files << region_file;
                upload_targets << remote_region;
            }
            
            // 上传文件
            bool upload_success = true;
            for (int i = 0; i < upload_files.size(); ++i) {
                updateStatus(QString("正在上传 %1/%2: %3").arg(i+1).arg(upload_files.size()).arg(QFileInfo(upload_files[i]).fileName()));
                
                if (!transferFileViaSCP(upload_files[i], upload_targets[i], true)) {
                    upload_success = false;
                    break;
                }
                
                // 等待传输完成
                if (scp_process_) {
                    scp_process_->waitForFinished(30000); // 30秒超时
                }
            }
            
            // 清理临时目录
            QDir temp_dir_obj(temp_dir);
            temp_dir_obj.removeRecursively();
            
            // 显示结果
            QString ssh_path = QString("%1@%2:%3").arg(remote_user).arg(remote_host).arg(remote_dir);
            QString message = "上传到SSH: " + ssh_path + "\n\n";
            if (!saved_files.isEmpty())
            {
                message += "✓ 成功上传: " + saved_files.join(", ") + "\n";
            }
            if (!failed_files.isEmpty())
            {
                message += "✗ 上传失败: " + failed_files.join(", ") + "\n";
            }
            
            if (upload_success)
            {
                updateStatus("所有文件已上传到远程: " + remote_name);
                QMessageBox::information(this, "上传成功", message);
                
                // 自动刷新远程文件列表
                refreshRemoteFileList();
            }
            else
            {
                updateStatus("部分文件上传失败");
                QMessageBox::warning(this, "上传部分失败", message);
            }
        }
        else
        {
            // 清理临时目录
            QDir temp_dir_obj(temp_dir);
            temp_dir_obj.removeRecursively();
            
            QString message = "本地文件准备失败:\n✗ " + failed_files.join(", ");
            updateStatus("准备上传文件失败");
            QMessageBox::critical(this, "准备失败", message);
        }
    }
    catch (const std::exception& e)
    {
        QString error_msg = "上传过程中出现错误: " + QString::fromStdString(e.what());
        updateStatus("上传失败");
        QMessageBox::critical(this, "上传错误", error_msg);
    }
}

std::string MapEditPanel::getCurrentMapFile()
{
  // 首先检查用户是否手动加载了地图文件
  if (!current_map_file_.isEmpty())
  {
    return current_map_file_.toStdString();
  }
  
  // 从参数服务器获取当前地图文件路径
  ros::NodeHandle nh;
  std::string map_file;
  if (nh.getParam("/map_server/map_file", map_file))
  {
    return map_file;
  }
  
  return "";
}

void MapEditPanel::openMap()
{
  // 设置默认路径为ros_map_edit/maps目录
  QString default_path = "src/ros_map_edit/maps";
  
  // 如果目录不存在，尝试其他可能的路径
  QDir maps_dir(default_path);
  if (!maps_dir.exists()) {
    // 尝试相对于工作目录的路径
    QStringList possible_paths = {
      "ros_map_edit/maps",
      "../src/ros_map_edit/maps", 
      "../../src/ros_map_edit/maps",
      QDir::homePath() + "/ros_ws/cursor_ws/src/ros_map_edit/maps"
    };
    
    for (const QString& path : possible_paths) {
      if (QDir(path).exists()) {
        default_path = path;
        break;
      }
    }
  }
  
  QString filename = QFileDialog::getOpenFileName(this,
                                                  "打开地图文件",
                                                  default_path,
                                                  "YAML files (*.yaml);;PGM files (*.pgm);;All files (*.*)");
  
  if (!filename.isEmpty())
  {
    current_map_file_ = filename;
    
    // 先清空所有消息，再加载并发布新地图
    clearAllMessages();
    loadAndPublishMap(filename.toStdString());
    
    // 更新当前地图显示
    QString display_name = QFileInfo(filename).fileName();
    current_map_label_->setText("当前地图: " + display_name);
    current_map_label_->setStyleSheet("QLabel { color: #007700; font-weight: bold; padding: 5px; }");
  }
}

void MapEditPanel::loadAndPublishMap(const std::string& filename)
{
  try
  {
    nav_msgs::OccupancyGrid map;
    
    ROS_INFO("开始加载地图文件: %s", filename.c_str());
    status_label_->setText("正在加载地图: " + QString::fromStdString(filename));
    
    // 根据文件扩展名决定加载方式
    std::string ext = filename.substr(filename.find_last_of(".") + 1);
    ROS_INFO("检测到文件扩展名: %s", ext.c_str());
    
    if (ext == "yaml" || ext == "yml")
    {
      // 加载YAML配置的地图
      ROS_INFO("尝试加载YAML格式地图...");
      if (file_manager_->loadMap(filename, map))
      {
        ROS_INFO("地图加载成功: %dx%d像素, 分辨率: %.3f m/pixel", 
                 map.info.width, map.info.height, map.info.resolution);
        publishMap(map);
        
        // 地图加载成功后，清空并重新加载对应的虚拟墙和区域
        loadCorrespondingFiles(filename);
        
        status_label_->setText("地图已加载: " + QFileInfo(QString::fromStdString(filename)).fileName());
      }
      else
      {
        QString error = QString::fromStdString(file_manager_->getLastError());
        ROS_ERROR("地图加载失败: %s", error.toStdString().c_str());
        status_label_->setText("加载失败: " + error);
      }
    }
    else if (ext == "pgm")
    {
      // 直接加载PGM文件
      ROS_INFO("尝试加载PGM格式地图...");
      if (file_manager_->loadMap(filename, map))
      {
        ROS_INFO("地图加载成功: %dx%d像素, 分辨率: %.3f m/pixel", 
                 map.info.width, map.info.height, map.info.resolution);
        publishMap(map);
        
        // 地图加载成功后，清空并重新加载对应的虚拟墙和区域
        loadCorrespondingFiles(filename);
        
        status_label_->setText("地图已加载: " + QFileInfo(QString::fromStdString(filename)).fileName());
      }
      else
      {
        QString error = QString::fromStdString(file_manager_->getLastError());
        ROS_ERROR("地图加载失败: %s", error.toStdString().c_str());
        status_label_->setText("加载失败: " + error);
      }
    }
    else
    {
      QString error = "不支持的文件格式: " + QString::fromStdString(ext);
      ROS_ERROR("%s", error.toStdString().c_str());
      status_label_->setText(error);
    }
  }
  catch (const std::exception& e)
  {
    QString error = "加载出错: " + QString::fromStdString(e.what());
    ROS_ERROR("%s", error.toStdString().c_str());
    status_label_->setText(error);
  }
}

void MapEditPanel::publishMap(const nav_msgs::OccupancyGrid& map)
{
  // 创建地图发布器 - 使用静态发布器确保持续发布
  static ros::Publisher map_pub;
  static bool initialized = false;
  
  if (!initialized)
  {
    ros::NodeHandle nh;
    // 使用latched=true确保新订阅者能立即收到地图数据
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    initialized = true;
    
    // 等待发布器准备就绪
    ros::Duration(0.5).sleep();
  }
  
  // 设置地图头信息
  nav_msgs::OccupancyGrid map_msg = map;
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = "map";
  
  // 发布地图多次以确保被接收
  for (int i = 0; i < 3; ++i)
  {
    map_pub.publish(map_msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  
  // 输出调试信息
  QString debug_info = QString("地图已发布: %1x%2像素, 分辨率: %3m/pixel")
                      .arg(map_msg.info.width)
                      .arg(map_msg.info.height)
                      .arg(map_msg.info.resolution);
  
  status_label_->setText(debug_info);
  
  // 同时发布到map_metadata话题
  static ros::Publisher metadata_pub;
  static bool metadata_initialized = false;
  
  if (!metadata_initialized)
  {
    ros::NodeHandle nh;
    metadata_pub = nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    metadata_initialized = true;
  }
  
  metadata_pub.publish(map_msg.info);
  
  ROS_INFO("地图已发布: %dx%d, 分辨率: %.3f", 
           map_msg.info.width, map_msg.info.height, map_msg.info.resolution);
}

void MapEditPanel::loadCorrespondingFiles(const std::string& map_file_path)
{
  // 清空并重新加载对应的虚拟墙和区域文件
  try
  {
    // 获取工具管理器实例
    ToolManager& toolManager = ToolManager::getInstance();
    
    // 清空并重新加载虚拟墙
    VirtualWallTool* wallTool = toolManager.getVirtualWallTool();
    if (wallTool)
    {
      ROS_INFO("清空并重新加载虚拟墙文件...");
      wallTool->loadVirtualWallsForMap(map_file_path);
    }
    
    // 清空并重新加载区域
    RegionTool* regionTool = toolManager.getRegionTool();
    if (regionTool)
    {
      ROS_INFO("清空并重新加载区域文件...");
      regionTool->loadRegionsForMap(map_file_path);
    }
    
    ROS_INFO("已清空并重新加载地图对应的虚拟墙和区域文件");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("加载对应文件时出错: %s", e.what());
  }
}

void MapEditPanel::clearAllMessages()
{
  // 清空所有ROS话题消息，确保加载新地图时不会显示旧数据
  try
  {
    ROS_INFO("正在清空所有消息...");
    status_label_->setText("正在清空现有数据...");
    
    // 1. 清空地图消息 - 发布空地图
    static ros::Publisher map_pub;
    static ros::Publisher metadata_pub; 
    static ros::Publisher edited_map_pub;
    static bool publishers_initialized = false;
    
    if (!publishers_initialized)
    {
      ros::NodeHandle nh;
      map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
      metadata_pub = nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
      edited_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_edited", 1, true);
      publishers_initialized = true;
      
      // 等待发布器准备就绪
      ros::Duration(0.2).sleep();
    }
    
    // 创建空地图消息
    nav_msgs::OccupancyGrid empty_map;
    empty_map.header.stamp = ros::Time::now();
    empty_map.header.frame_id = "map";
    empty_map.info.width = 0;
    empty_map.info.height = 0;
    empty_map.info.resolution = 0.05;
    empty_map.info.origin.position.x = 0.0;
    empty_map.info.origin.position.y = 0.0;
    empty_map.info.origin.position.z = 0.0;
    empty_map.info.origin.orientation.w = 1.0;
    empty_map.data.clear();
    
    // 发布空地图消息
    for (int i = 0; i < 3; ++i)
    {
      map_pub.publish(empty_map);
      edited_map_pub.publish(empty_map);
      metadata_pub.publish(empty_map.info);
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
    
    // 2. 先发布DELETE_ALL消息清除所有标记，然后清空虚拟墙和区域数据
    static ros::Publisher wall_marker_pub;
    static ros::Publisher region_marker_pub;
    static bool marker_publishers_initialized = false;
    
    if (!marker_publishers_initialized)
    {
      ros::NodeHandle nh;
      wall_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("virtual_walls_markers", 1, true);
      region_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("region_markers", 1, true);
      marker_publishers_initialized = true;
      
      ros::Duration(0.1).sleep();
    }
    
    // 创建DELETE_ALL标记来清除所有现有的虚拟墙和区域标记
    visualization_msgs::MarkerArray delete_all_markers;
    
    // 创建删除所有虚拟墙标记的消息
    visualization_msgs::Marker delete_wall_marker;
    delete_wall_marker.header.frame_id = "map";
    delete_wall_marker.header.stamp = ros::Time::now();
    delete_wall_marker.ns = "virtual_walls";
    delete_wall_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_all_markers.markers.push_back(delete_wall_marker);
    
    // 创建删除所有区域标记的消息
    visualization_msgs::Marker delete_region_marker;
    delete_region_marker.header.frame_id = "map";
    delete_region_marker.header.stamp = ros::Time::now();
    delete_region_marker.ns = "regions";
    delete_region_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_all_markers.markers.push_back(delete_region_marker);
    
    // 添加删除虚拟墙点标记的消息
    visualization_msgs::Marker delete_wall_points_marker;
    delete_wall_points_marker.header.frame_id = "map";
    delete_wall_points_marker.header.stamp = ros::Time::now();
    delete_wall_points_marker.ns = "virtual_wall_points";
    delete_wall_points_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_all_markers.markers.push_back(delete_wall_points_marker);
    
    // 添加删除当前墙标记的消息
    visualization_msgs::Marker delete_current_wall_marker;
    delete_current_wall_marker.header.frame_id = "map";
    delete_current_wall_marker.header.stamp = ros::Time::now();
    delete_current_wall_marker.ns = "current_wall";
    delete_current_wall_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_all_markers.markers.push_back(delete_current_wall_marker);
    
    // 发布DELETE_ALL消息
    for (int i = 0; i < 5; ++i)
    {
      wall_marker_pub.publish(delete_all_markers);
      region_marker_pub.publish(delete_all_markers);
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
    
    ROS_INFO("发布了DELETE_ALL标记消息");
    
    // 3. 清空虚拟墙和区域数据
    ToolManager& toolManager = ToolManager::getInstance();
    
    // 清空虚拟墙
    VirtualWallTool* wallTool = toolManager.getVirtualWallTool();
    if (wallTool)
    {
      ROS_INFO("清空虚拟墙数据...");
      wallTool->clearVirtualWalls();
    }
    
    // 清空区域
    RegionTool* regionTool = toolManager.getRegionTool();
    if (regionTool)
    {
      ROS_INFO("清空区域数据...");
      regionTool->clearRegions();
    }
    
    // 4. 最后发布空的marker数组确保清空
    visualization_msgs::MarkerArray empty_markers;
    empty_markers.markers.clear();
    
    // 发布空marker数组
    for (int i = 0; i < 3; ++i)
    {
      wall_marker_pub.publish(empty_markers);
      region_marker_pub.publish(empty_markers);
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
    
    ROS_INFO("所有消息已清空");
    status_label_->setText("现有数据已清空");
    
    // 短暂等待确保消息被接收
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("清空消息时出错: %s", e.what());
    status_label_->setText("清空数据时出错");
  }
}

void MapEditPanel::openLocalMap()
{
  openMap(); // 调用原有的本地文件打开函数
}

void MapEditPanel::openRemoteMap()
{
  // 此函数现在由选项卡自动处理，无需额外实现
}

void MapEditPanel::setPassword()
{
    // 弹出密码输入对话框
    bool ok;
    QString password = QInputDialog::getText(
        this,
        "SSH密码认证",
        "请输入远程服务器密码:",
        QLineEdit::Password,  // 密码输入模式
        "",
        &ok
    );
    
    if(ok && !password.isEmpty()) {
        current_password_ = password;
        updateStatus("密码已设置");
        
        // 更新按钮状态
        bool has_config = !ssh_host_edit_->text().isEmpty() && 
                         !ssh_username_edit_->text().isEmpty() &&
                         !ssh_remote_dir_edit_->text().isEmpty();
        test_connection_btn_->setEnabled(has_config);
        save_to_remote_btn_->setEnabled(has_config);
    }
}

void MapEditPanel::testSSHConnection()
{
    // 获取远程配置
    QString remote_host = ssh_host_edit_->text().trimmed();
    QString remote_user = ssh_username_edit_->text().trimmed();
    QString remote_dir = ssh_remote_dir_edit_->text().trimmed();
    
    // 验证配置
    if(remote_host.isEmpty() || remote_user.isEmpty()) {
        updateStatus("错误: SSH配置不完整", true);
        QMessageBox::warning(this, "警告", "请输入完整的SSH服务器地址和用户名");
        return;
    }
    
    // 检查密码是否设置
    if(current_password_.isEmpty()) {
        updateStatus("错误: 未设置密码", true);
        QMessageBox::warning(this, "警告", "请先设置SSH密码");
        return;
    }
    
    updateStatus("正在测试SSH连接...");
    connection_status_label_->setText("正在测试连接...");
    connection_status_label_->setStyleSheet("QLabel { color: #FF9800; font-weight: bold; }");
    
    // 创建SSH测试进程
    QProcess* test_process = new QProcess(this);
    test_process->setProcessChannelMode(QProcess::MergedChannels);
    
    // 构建SSH测试命令 - 尝试列出远程目录
    QString ssh_command = QString("sshpass -p '%1' ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 %2@%3 'ls -la %4'")
                         .arg(current_password_)
                         .arg(remote_user)
                         .arg(remote_host)
                         .arg(remote_dir);
    
    connect(test_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            [this, test_process, remote_host, remote_user, remote_dir](int exitCode, QProcess::ExitStatus exitStatus) {
        if(exitCode == 0) {
            updateStatus("SSH连接测试成功");
            connection_status_label_->setText("连接成功!");
            connection_status_label_->setStyleSheet("QLabel { color: #4CAF50; font-weight: bold; }");
            refresh_files_btn_->setEnabled(true);
            
            QMessageBox::information(this, "连接测试", 
                                   QString("SSH连接测试成功！\n\n服务器: %1@%2\n目录: %3\n\n可以正常访问远程服务器。")
                                   .arg(remote_user)
                                   .arg(remote_host)
                                   .arg(remote_dir));
            
            // 自动刷新文件列表
            refreshRemoteFileList();
        } else {
            QString errorOutput = test_process->readAll();
            QString errorMsg = QString("SSH连接测试失败 (退出码: %1)").arg(exitCode);
            if(!errorOutput.isEmpty()) {
                errorMsg += "\n\n详细信息:\n" + errorOutput;
            }
            updateStatus("SSH连接测试失败", true);
            connection_status_label_->setText("连接失败");
            connection_status_label_->setStyleSheet("QLabel { color: #f44336; font-weight: bold; }");
            refresh_files_btn_->setEnabled(false);
            download_open_btn_->setEnabled(false);
            
            QMessageBox::critical(this, "连接测试失败", errorMsg);
        }
        
        test_process->deleteLater();
    });
    
    connect(test_process, &QProcess::errorOccurred, [this, test_process](QProcess::ProcessError error){
        QString errorMsg;
        switch(error) {
            case QProcess::FailedToStart: errorMsg = "进程启动失败，请确保已安装sshpass"; break;
            case QProcess::Crashed: errorMsg = "进程崩溃"; break;
            case QProcess::Timedout: errorMsg = "操作超时"; break;
            default: errorMsg = "未知错误";
        }
        updateStatus("SSH测试错误: " + errorMsg, true);
        connection_status_label_->setText("测试失败");
        connection_status_label_->setStyleSheet("QLabel { color: #f44336; font-weight: bold; }");
        
        QMessageBox::critical(this, "SSH测试错误", 
                            QString("SSH连接测试失败！\n\n错误: %1")
                            .arg(errorMsg));
        test_process->deleteLater();
    });
    
    ROS_INFO_STREAM("Testing SSH connection to: " << remote_user.toStdString() << "@" << remote_host.toStdString());
    test_process->start("bash", QStringList() << "-c" << ssh_command);
}

void MapEditPanel::refreshRemoteFileList()
{
    remote_files_list_->clear();
    
    QString remote_host = ssh_host_edit_->text().trimmed();
    QString remote_user = ssh_username_edit_->text().trimmed();
    QString remote_dir = ssh_remote_dir_edit_->text().trimmed();
    
    if(current_password_.isEmpty()) {
        QListWidgetItem* item = new QListWidgetItem("请先设置密码");
        item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
        remote_files_list_->addItem(item);
        return;
    }
    
    updateStatus("正在获取远程文件列表...");
    
    // 创建SSH进程获取文件列表
    QProcess* list_process = new QProcess(this);
    list_process->setProcessChannelMode(QProcess::MergedChannels);
    
    // 构建SSH命令获取地图文件列表
    QString ssh_command = QString("sshpass -p '%1' ssh -o StrictHostKeyChecking=no %2@%3 'find %4 -name \"*.yaml\" -o -name \"*.yml\" -o -name \"*.pgm\" | head -20'")
                         .arg(current_password_)
                         .arg(remote_user)
                         .arg(remote_host)
                         .arg(remote_dir);
    
    connect(list_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            [this, list_process](int exitCode, QProcess::ExitStatus exitStatus) {
        if(exitCode == 0) {
            QString output = list_process->readAll();
            QStringList files = output.split('\n', QString::SkipEmptyParts);
            
            if(files.isEmpty()) {
                QListWidgetItem* item = new QListWidgetItem("未找到地图文件");
                item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
                remote_files_list_->addItem(item);
            } else {
                for(const QString& file : files) {
                    QFileInfo fileInfo(file.trimmed());
                    remote_files_list_->addItem(fileInfo.fileName());
                }
                updateStatus(QString("找到 %1 个地图文件").arg(files.size()));
            }
        } else {
            QString errorOutput = list_process->readAll();
            QListWidgetItem* item = new QListWidgetItem("获取文件列表失败: " + errorOutput);
            item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
            remote_files_list_->addItem(item);
            updateStatus("获取文件列表失败", true);
        }
        
        list_process->deleteLater();
    });
    
    connect(list_process, &QProcess::errorOccurred, [this, list_process](QProcess::ProcessError error){
        QListWidgetItem* item = new QListWidgetItem("获取文件列表时出错");
        item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
        remote_files_list_->addItem(item);
        updateStatus("获取文件列表时出错", true);
        list_process->deleteLater();
    });
    
    list_process->start("bash", QStringList() << "-c" << ssh_command);
}

void MapEditPanel::onRemoteFileSelected()
{
    QListWidgetItem* current_item = remote_files_list_->currentItem();
    if (current_item && (current_item->flags() & Qt::ItemIsSelectable)) {
        download_open_btn_->setEnabled(true);
        QString filename = current_item->text();
        updateStatus("已选择: " + filename);
    } else {
        download_open_btn_->setEnabled(false);
    }
}

void MapEditPanel::downloadAndOpenRemoteMap()
{
    QListWidgetItem* current_item = remote_files_list_->currentItem();
    if (!current_item || !(current_item->flags() & Qt::ItemIsSelectable)) {
        QMessageBox::warning(this, "警告", "请先选择一个地图文件");
        return;
    }
    
    QString remote_filename = current_item->text();
    QString remote_host = ssh_host_edit_->text().trimmed();
    QString remote_user = ssh_username_edit_->text().trimmed();
    QString remote_dir = ssh_remote_dir_edit_->text().trimmed();
    
    // 检查是否为YAML文件
    if (!remote_filename.endsWith(".yaml") && !remote_filename.endsWith(".yml")) {
        QMessageBox::warning(this, "警告", "请选择YAML地图文件(.yaml或.yml)");
        return;
    }
    
    updateStatus("正在从远程服务器下载地图文件集...");
    
    // 显示进度对话框
    download_progress_->setVisible(true);
    download_progress_->setRange(0, 100);
    download_progress_->setValue(0);
    download_open_btn_->setEnabled(false);
    
    // 创建临时目录来保存下载的文件
    QString temp_dir = QDir::tempPath() + "/ros_map_download_" + 
                      QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QDir().mkpath(temp_dir);
    current_temp_file_ = temp_dir; // 保存临时目录路径
    
    // 生成所有文件名
    QString base_name = remote_filename;
    if(base_name.endsWith(".yaml")) {
        base_name = base_name.left(base_name.length() - 5);
    } else if(base_name.endsWith(".yml")) {
        base_name = base_name.left(base_name.length() - 4);
    }
    
    QString yaml_filename = remote_filename;
    QString pgm_filename = base_name + ".pgm";
    QString json_filename = base_name + ".json";
    QString region_filename = base_name + "_region.json";
    
    // 开始下载流程：YAML -> PGM -> JSON -> region.json -> 完成
    downloadMapFileSequence(temp_dir, remote_host, remote_user, remote_dir, 
                            yaml_filename, pgm_filename, json_filename, region_filename, 0);
}

void MapEditPanel::downloadMapFileSequence(const QString& temp_dir, const QString& remote_host, 
                                          const QString& remote_user, const QString& remote_dir,
                                          const QString& yaml_filename, const QString& pgm_filename,
                                          const QString& json_filename, const QString& region_filename,
                                          int step)
{
    QStringList file_list = {yaml_filename, pgm_filename, json_filename, region_filename};
    QStringList step_names = {"YAML配置", "PGM图像", "虚拟墙JSON", "区域JSON"};
    QStringList file_descriptions = {"地图配置文件", "地图图像文件", "虚拟墙文件(可选)", "区域文件(可选)"};
    
    if(step >= file_list.size()) {
        // 所有文件下载完成
        onAllFilesDownloaded(temp_dir, yaml_filename);
        return;
    }
    
    QString current_file = file_list[step];
    QString step_name = step_names[step];
    QString file_desc = file_descriptions[step];
    
    // 更新进度
    int progress = (step * 100) / file_list.size();
    download_progress_->setValue(progress);
    updateStatus(QString("正在下载%1: %2...").arg(file_desc).arg(current_file));
    
    QString local_path = temp_dir + "/" + current_file;
    QString remote_path = QString("%1/%2").arg(remote_dir).arg(current_file);
    QString remote_source = QString("%1@%2:%3").arg(remote_user).arg(remote_host).arg(remote_path);
    
    // 开始下载当前文件
    if(transferFileViaSCP(remote_source, local_path, false)) {
        // 下载完成后的处理
        connect(scp_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, [this, temp_dir, remote_host, remote_user, remote_dir, 
                      yaml_filename, pgm_filename, json_filename, region_filename, 
                      step, current_file, step_name, file_desc](int exitCode, QProcess::ExitStatus exitStatus) {
            
            if(exitCode == 0) {
                ROS_INFO("成功下载%s: %s", file_desc.toStdString().c_str(), current_file.toStdString().c_str());
                
                // 如果是YAML文件，需要修改其中的image路径
                if(step == 0) { // YAML文件
                    QString local_yaml = temp_dir + "/" + yaml_filename;
                    QFile yaml_file(local_yaml);
                    if(yaml_file.open(QIODevice::ReadWrite)) {
                        QString yaml_content = yaml_file.readAll();
                        yaml_file.close();
                        
                        // 替换image行，将其改为相对路径
                        QStringList lines = yaml_content.split('\n');
                        for(int i = 0; i < lines.size(); ++i) {
                            QString line = lines[i].trimmed();
                            if(line.startsWith("image:")) {
                                lines[i] = "image: " + pgm_filename;
                                break;
                            }
                        }
                        
                        // 重写YAML文件
                        if(yaml_file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                            yaml_file.write(lines.join('\n').toUtf8());
                            yaml_file.close();
                            ROS_INFO("已修改YAML文件，将image路径设置为: %s", pgm_filename.toStdString().c_str());
                        }
                    }
                }
                
                // 继续下载下一个文件
                downloadMapFileSequence(temp_dir, remote_host, remote_user, remote_dir,
                                       yaml_filename, pgm_filename, json_filename, region_filename, step + 1);
            } else {
                // 当前文件下载失败
                if(step < 2) { // YAML和PGM是必需的
                    download_progress_->setVisible(false);
                    download_open_btn_->setEnabled(true);
                    QDir(temp_dir).removeRecursively();
                    current_temp_file_.clear();
                    updateStatus(QString("%1下载失败").arg(file_desc), true);
                    QMessageBox::critical(this, "下载失败", QString("%1下载失败，无法加载地图").arg(file_desc));
                } else {
                    // JSON文件下载失败，给出友好提示并继续
                    ROS_INFO("可选文件不存在或下载失败: %s (这是正常的，该文件可能不存在)", current_file.toStdString().c_str());
                    updateStatus(QString("%1不存在，跳过").arg(file_desc));
                    // 继续下载下一个文件
                    downloadMapFileSequence(temp_dir, remote_host, remote_user, remote_dir,
                                           yaml_filename, pgm_filename, json_filename, region_filename, step + 1);
                }
            }
        });
    } else {
        // SCP启动失败
        if(step < 2) { // 必需文件启动失败
            download_progress_->setVisible(false);
            download_open_btn_->setEnabled(true);
            QDir(temp_dir).removeRecursively();
            current_temp_file_.clear();
            updateStatus(QString("无法启动%1传输").arg(file_desc), true);
        } else {
            // 可选文件启动失败，继续下一个
            ROS_INFO("可选文件传输启动失败，跳过: %s", current_file.toStdString().c_str());
            updateStatus(QString("%1传输跳过").arg(file_desc));
            downloadMapFileSequence(temp_dir, remote_host, remote_user, remote_dir,
                                   yaml_filename, pgm_filename, json_filename, region_filename, step + 1);
        }
    }
}

void MapEditPanel::onAllFilesDownloaded(const QString& temp_dir, const QString& yaml_filename)
{
    download_progress_->setValue(100);
    download_progress_->setVisible(false);
    download_open_btn_->setEnabled(true);
    
    QString local_yaml = temp_dir + "/" + yaml_filename;
    
    // 验证核心文件
    QFileInfo yaml_info(local_yaml);
    if(!yaml_info.exists() || yaml_info.size() == 0) {
        updateStatus("错误: YAML文件下载不完整", true);
        QMessageBox::critical(this, "下载失败", "YAML文件下载不完整");
        QDir(temp_dir).removeRecursively();
        current_temp_file_.clear();
        return;
    }
    
    // 检查并统计下载的文件
    QString base_name = yaml_filename;
    if(base_name.endsWith(".yaml")) {
        base_name = base_name.left(base_name.length() - 5);
    } else if(base_name.endsWith(".yml")) {
        base_name = base_name.left(base_name.length() - 4);
    }
    
    QStringList check_files = {
        yaml_filename,
        base_name + ".pgm",
        base_name + ".json", 
        base_name + "_region.json"
    };
    QStringList file_types = {"地图配置文件", "地图图像文件", "虚拟墙文件", "区域数据文件"};
    QStringList required_flags = {"必需", "必需", "可选", "可选"};
    
    QStringList downloaded_files;
    QStringList missing_optional_files;
    
    for(int i = 0; i < check_files.size(); ++i) {
        QFileInfo file_info(temp_dir + "/" + check_files[i]);
        if(file_info.exists() && file_info.size() > 0) {
            downloaded_files << file_types[i];
        } else {
            if(i >= 2) { // 可选文件
                missing_optional_files << file_types[i];
            }
        }
    }
    
    // 验证必需的PGM文件
    QString pgm_path = temp_dir + "/" + base_name + ".pgm";
    QFileInfo pgm_info(pgm_path);
    if(!pgm_info.exists() || pgm_info.size() == 0) {
        updateStatus("错误: PGM图像文件下载不完整", true);
        QMessageBox::critical(this, "下载失败", "PGM图像文件下载不完整，无法加载地图");
        QDir(temp_dir).removeRecursively();
        current_temp_file_.clear();
        return;
    }
    
    // 文件验证通过，加载地图
    // 存储实际的临时YAML文件路径，而不是带前缀的名称
    current_map_file_ = local_yaml;
    is_remote_map_ = true;
    // 存储原始的远程文件名，用于后续操作
    remote_map_base_name_ = base_name;
    
    // 清空现有数据并加载新地图
    clearAllMessages();
    loadAndPublishMap(local_yaml.toStdString());
    
    // 更新UI显示
    current_map_label_->setText("当前地图: " + yaml_filename + " (远程)");
    current_map_label_->setStyleSheet("QLabel { color: #007700; font-weight: bold; padding: 5px; }");
    updateStatus("远程地图文件集下载成功");
    
    // 构建详细的成功消息
    QString success_msg = QString("地图文件集下载成功!\n\n远程文件: %1\n本地临时目录: %2\n\n").arg(yaml_filename).arg(temp_dir);
    
    if(!downloaded_files.isEmpty()) {
        success_msg += "✓ 已下载: " + downloaded_files.join(", ") + "\n";
    }
    
    if(!missing_optional_files.isEmpty()) {
        success_msg += "ⓘ 未找到可选文件: " + missing_optional_files.join(", ") + " (这是正常的)\n";
    }
    
    success_msg += "\n地图已成功加载到RViz中。";
    
    QMessageBox::information(this, "下载成功", success_msg);
    
    // 保留临时文件用于后续保存操作
}

bool MapEditPanel::transferFileViaSCP(const QString& source, const QString& target, bool is_upload)
{
    // 检查密码是否设置
    if(current_password_.isEmpty()) {
        updateStatus("错误: 未设置密码", true);
        QMessageBox::warning(this, "警告", "请先设置密码");
        return false;
    }

    // 如果已有传输在进行，则终止
    if(scp_process_) {
        if(scp_process_->state() == QProcess::Running) {
            scp_process_->terminate();
            if(!scp_process_->waitForFinished(2000)) {
                scp_process_->kill();
            }
        }
        scp_process_->deleteLater();
        scp_process_ = nullptr;
    }

    // 创建新的SCP进程
    scp_process_ = new QProcess(this);
    scp_process_->setProcessChannelMode(QProcess::MergedChannels);
    
    // 构建SCP命令 - 使用sshpass自动输入密码
    QString scp_command = QString("sshpass -p '%1' scp -o StrictHostKeyChecking=no -o ConnectTimeout=15 -o ServerAliveInterval=60 %2 %3")
                         .arg(current_password_)
                         .arg(source)  // 源文件
                         .arg(target); // 目标文件

    // 设置进度对话框
    if(!progress_dialog_) {
        progress_dialog_ = new QProgressDialog(this);
        progress_dialog_->setWindowModality(Qt::WindowModal);
        progress_dialog_->setCancelButton(nullptr);
        progress_dialog_->setMinimumDuration(0);
    }
    
    progress_dialog_->setLabelText(is_upload ? "正在上传到远程服务器..." : "正在从远程下载...");
    progress_dialog_->setRange(0, 100);
    progress_dialog_->setValue(0);
    progress_dialog_->show();

    // 使用成员变量存储状态
    scp_password_sent_ = false;
    scp_output_.clear();

    // 连接信号槽
    connect(scp_process_, &QProcess::readyRead, [this](){
        QString output = QString(scp_process_->readAll());
        scp_output_ += output;
        
        // 调试输出
        ROS_DEBUG_STREAM("SCP output: " << output.toStdString());
        
        // 解析进度百分比
        QRegExp progress_regex("(\\d+)%");
        if(progress_regex.indexIn(output) != -1) {
            int percent = progress_regex.cap(1).toInt();
            progress_dialog_->setValue(percent);
        }
    });

    connect(scp_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MapEditPanel::handleScpFinished);
    
    connect(scp_process_, &QProcess::errorOccurred, this, &MapEditPanel::handleScpError);

    // 启动SCP进程
    ROS_INFO_STREAM("Executing SCP command: scp " << (is_upload ? source.toStdString() : target.toStdString()) 
                   << " -> " << (is_upload ? target.toStdString() : source.toStdString()));
    
    scp_process_->start("bash", QStringList() << "-c" << scp_command);
    
    if(!scp_process_->waitForStarted(3000)) {
        updateStatus("错误: 无法启动SCP进程", true);
        ROS_ERROR("Failed to start SCP process");
        return false;
    }
    
    ROS_INFO("SCP process started successfully");
    return true;
}

void MapEditPanel::handleScpFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    if(progress_dialog_) {
        progress_dialog_->reset();
    }
    
    if(exitCode == 0) {
        updateStatus("文件传输成功完成");
        ROS_INFO("SCP传输成功完成");
    } else {
        QString errorOutput = scp_process_->readAllStandardError();
        QString standardOutput = scp_process_->readAllStandardOutput();
        
        if(errorOutput.isEmpty() && standardOutput.isEmpty() && !scp_output_.isEmpty()) {
            errorOutput = scp_output_;
        }
        
        if(errorOutput.isEmpty() && standardOutput.isEmpty()) {
            errorOutput = "无详细错误信息";
        }
        
        QString errorMsg = QString("文件传输失败 (退出码: %1)").arg(exitCode);
        if(!errorOutput.isEmpty()) {
            errorMsg += "\n错误输出: " + errorOutput;
        }
        if(!standardOutput.isEmpty()) {
            errorMsg += "\n标准输出: " + standardOutput;
        }
        
        updateStatus(errorMsg, true);
        ROS_ERROR_STREAM("SCP failed: " << errorMsg.toStdString());
        
        // 显示详细错误对话框
        QMessageBox::critical(this, "SCP传输失败", 
                            QString("文件传输失败！\n\n退出码: %1\n\n详细信息:\n%2")
                            .arg(exitCode)
                            .arg(errorOutput.isEmpty() ? standardOutput : errorOutput));
    }
    
    if(scp_process_) {
        scp_process_->deleteLater();
        scp_process_ = nullptr;
    }
}

void MapEditPanel::handleScpError(QProcess::ProcessError error)
{
    if(progress_dialog_) {
        progress_dialog_->reset();
    }
    
    QString errorMsg;
    switch(error) {
        case QProcess::FailedToStart: errorMsg = "进程启动失败，请确保已安装sshpass"; break;
        case QProcess::Crashed: errorMsg = "进程崩溃"; break;
        case QProcess::Timedout: errorMsg = "操作超时"; break;
        case QProcess::WriteError: errorMsg = "写入错误"; break;
        case QProcess::ReadError: errorMsg = "读取错误"; break;
        default: errorMsg = "未知错误";
    }
    
    updateStatus("SCP错误: " + errorMsg, true);
    ROS_ERROR_STREAM("SCP process error: " << errorMsg.toStdString() << "\nFull output: " << scp_output_.toStdString());
    
    QMessageBox::critical(this, "SCP传输错误", 
                        QString("SCP传输失败！\n\n错误: %1\n\n建议:\n1. 确保已安装sshpass\n2. 检查网络连接\n3. 验证SSH配置")
                        .arg(errorMsg));
    
    if(scp_process_) {
        scp_process_->deleteLater();
        scp_process_ = nullptr;
    }
}

QString MapEditPanel::generateTempFilePath() const
{
    // 生成带时间戳的临时目录路径
    return QDir::tempPath() + "/ros_map_" + 
           QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
}

void MapEditPanel::updateStatus(const QString& message, bool is_error)
{
    // 更新状态标签，错误消息显示为红色
    QString style = is_error ? "color: red;" : "color: black;";
    status_label_->setText("<span style='" + style + "'>" + message + "</span>");
}

void MapEditPanel::saveConnectionSettings()
{
    // 保存SSH配置到QSettings
    QSettings settings;
    settings.setValue("ssh_host", ssh_host_edit_->text());
    settings.setValue("ssh_port", ssh_port_edit_->text());
    settings.setValue("ssh_username", ssh_username_edit_->text());
    settings.setValue("ssh_remote_dir", ssh_remote_dir_edit_->text());
}

void MapEditPanel::loadConnectionSettings()
{
    // 从QSettings加载SSH配置
    QSettings settings;
    ssh_host_edit_->setText(settings.value("ssh_host", "example.local").toString());
    ssh_port_edit_->setText(settings.value("ssh_port", "22").toString());
    ssh_username_edit_->setText(settings.value("ssh_username", "oryxbot").toString());
    ssh_remote_dir_edit_->setText(settings.value("ssh_remote_dir", "/home/oryxbot/.reinovo/maps").toString());
}

void MapEditPanel::clearVirtualWalls()
{
    try
    {
        // 确认操作
        QMessageBox::StandardButton reply = QMessageBox::question(
            this, 
            "确认清除", 
            "确定要清除所有虚拟墙吗？\n\n此操作无法撤销。",
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No
        );
        
        if (reply != QMessageBox::Yes) {
            return;
        }
        
        updateStatus("正在清除虚拟墙...");
        
        // 获取工具管理器实例
        ToolManager& toolManager = ToolManager::getInstance();
        
        // 清空虚拟墙数据
        VirtualWallTool* wallTool = toolManager.getVirtualWallTool();
        if (wallTool)
        {
            ROS_INFO("清除虚拟墙数据...");
            wallTool->clearVirtualWalls();
        }
        
        // 发布DELETE标记清除RViz中的显示
        static ros::Publisher wall_marker_pub;
        static bool wall_publisher_initialized = false;
        
        if (!wall_publisher_initialized)
        {
            ros::NodeHandle nh;
            wall_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("virtual_walls_markers", 1, true);
            wall_publisher_initialized = true;
            ros::Duration(0.1).sleep();
        }
        
        // 创建DELETE_ALL标记
        visualization_msgs::MarkerArray delete_markers;
        
        visualization_msgs::Marker delete_wall_marker;
        delete_wall_marker.header.frame_id = "map";
        delete_wall_marker.header.stamp = ros::Time::now();
        delete_wall_marker.ns = "virtual_walls";
        delete_wall_marker.action = visualization_msgs::Marker::DELETEALL;
        delete_markers.markers.push_back(delete_wall_marker);
        
        visualization_msgs::Marker delete_wall_points_marker;
        delete_wall_points_marker.header.frame_id = "map";
        delete_wall_points_marker.header.stamp = ros::Time::now();
        delete_wall_points_marker.ns = "virtual_wall_points";
        delete_wall_points_marker.action = visualization_msgs::Marker::DELETEALL;
        delete_markers.markers.push_back(delete_wall_points_marker);
        
        visualization_msgs::Marker delete_current_wall_marker;
        delete_current_wall_marker.header.frame_id = "map";
        delete_current_wall_marker.header.stamp = ros::Time::now();
        delete_current_wall_marker.ns = "current_wall";
        delete_current_wall_marker.action = visualization_msgs::Marker::DELETEALL;
        delete_markers.markers.push_back(delete_current_wall_marker);
        
        // 发布DELETE标记
        for (int i = 0; i < 3; ++i)
        {
            wall_marker_pub.publish(delete_markers);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        // 发布空的marker数组
        visualization_msgs::MarkerArray empty_markers;
        for (int i = 0; i < 3; ++i)
        {
            wall_marker_pub.publish(empty_markers);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        updateStatus("虚拟墙已清除");
        QMessageBox::information(this, "清除完成", "所有虚拟墙已成功清除");
        
        ROS_INFO("虚拟墙已清除完成");
        
        // 自动保存虚拟墙文件
        saveVirtualWallsFileAfterClear();
    }
    catch (const std::exception& e)
    {
        QString error_msg = "清除虚拟墙时出错: " + QString::fromStdString(e.what());
        updateStatus(error_msg, true);
        QMessageBox::critical(this, "清除失败", error_msg);
        ROS_ERROR("清除虚拟墙时出错: %s", e.what());
    }
}

void MapEditPanel::clearRegions()
{
    try
    {
        // 确认操作
        QMessageBox::StandardButton reply = QMessageBox::question(
            this, 
            "确认清除", 
            "确定要清除所有区域吗？\n\n此操作无法撤销。",
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No
        );
        
        if (reply != QMessageBox::Yes) {
            return;
        }
        
        updateStatus("正在清除区域...");
        
        // 获取工具管理器实例
        ToolManager& toolManager = ToolManager::getInstance();
        
        // 清空区域数据
        RegionTool* regionTool = toolManager.getRegionTool();
        if (regionTool)
        {
            ROS_INFO("清除区域数据...");
            regionTool->clearRegions();
        }
        
        // 发布DELETE标记清除RViz中的显示
        static ros::Publisher region_marker_pub;
        static bool region_publisher_initialized = false;
        
        if (!region_publisher_initialized)
        {
            ros::NodeHandle nh;
            region_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("region_markers", 1, true);
            region_publisher_initialized = true;
            ros::Duration(0.1).sleep();
        }
        
        // 创建DELETE_ALL标记
        visualization_msgs::MarkerArray delete_markers;
        
        visualization_msgs::Marker delete_region_marker;
        delete_region_marker.header.frame_id = "map";
        delete_region_marker.header.stamp = ros::Time::now();
        delete_region_marker.ns = "regions";
        delete_region_marker.action = visualization_msgs::Marker::DELETEALL;
        delete_markers.markers.push_back(delete_region_marker);
        
        // 发布DELETE标记
        for (int i = 0; i < 3; ++i)
        {
            region_marker_pub.publish(delete_markers);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        // 发布空的marker数组
        visualization_msgs::MarkerArray empty_markers;
        for (int i = 0; i < 3; ++i)
        {
            region_marker_pub.publish(empty_markers);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        updateStatus("区域已清除");
        QMessageBox::information(this, "清除完成", "所有区域已成功清除");
        
        ROS_INFO("区域已清除完成");
        
        // 自动保存区域文件
        saveRegionsFileAfterClear();
    }
    catch (const std::exception& e)
    {
        QString error_msg = "清除区域时出错: " + QString::fromStdString(e.what());
        updateStatus(error_msg, true);
        QMessageBox::critical(this, "清除失败", error_msg);
        ROS_ERROR("清除区域时出错: %s", e.what());
    }
}

void MapEditPanel::saveVirtualWallsFileAfterClear()
{
    try
    {
        // 获取当前地图文件路径
        std::string current_map_file = getCurrentMapFile();
        if (current_map_file.empty())
        {
            ROS_WARN("没有当前地图文件，无法自动保存虚拟墙文件");
            return;
        }
        
        // 构建虚拟墙文件路径
        QFileInfo map_info(QString::fromStdString(current_map_file));
        QString base_dir = map_info.absolutePath();
        QString base_name;
        
        // 如果是远程地图，使用存储的原始基础名称
        if (is_remote_map_ && !remote_map_base_name_.isEmpty()) {
            base_name = remote_map_base_name_;
        } else {
            base_name = map_info.baseName();
        }
        
        QString vw_file = base_dir + "/" + base_name + ".json";
        
        // 保存空的虚拟墙文件（清除后的状态）
        std::vector<VirtualWall> empty_walls;
        bool local_saved = false;
        
        // 1. 保存到本地
        if (file_manager_->saveVirtualWallsFile(vw_file.toStdString(), empty_walls))
        {
            ROS_INFO("虚拟墙文件已保存到本地: %s", vw_file.toStdString().c_str());
            local_saved = true;
        }
        else
        {
            ROS_WARN("保存本地虚拟墙文件失败: %s", file_manager_->getLastError().c_str());
        }
        
        // 2. 如果是远程地图，也保存到远程
        bool remote_saved = false;
        if (is_remote_map_ && !current_password_.isEmpty())
        {
            // 检查SSH连接状态
            if (connection_status_label_->text() == "连接成功!")
            {
                QString remote_host = ssh_host_edit_->text().trimmed();
                QString remote_user = ssh_username_edit_->text().trimmed();
                QString remote_dir = ssh_remote_dir_edit_->text().trimmed();
                
                // 构建远程文件路径
                QString remote_vw_file = QString("%1@%2:%3/%4.json")
                                        .arg(remote_user)
                                        .arg(remote_host)
                                        .arg(remote_dir)
                                        .arg(base_name);
                
                ROS_INFO("正在上传虚拟墙文件到远程: %s", remote_vw_file.toStdString().c_str());
                
                // 创建专用的SCP进程，避免与主进程冲突
                QProcess* upload_process = new QProcess(this);
                upload_process->setProcessChannelMode(QProcess::MergedChannels);
                
                // 构建SCP命令
                QString scp_command = QString("sshpass -p '%1' scp -o StrictHostKeyChecking=no -o ConnectTimeout=10 %2 %3")
                                     .arg(current_password_)
                                     .arg(vw_file)
                                     .arg(remote_vw_file);
                
                // 使用同步方式执行命令
                upload_process->start("bash", QStringList() << "-c" << scp_command);
                
                if (upload_process->waitForStarted(3000) && upload_process->waitForFinished(10000))
                {
                    if (upload_process->exitCode() == 0) {
                        remote_saved = true;
                        ROS_INFO("虚拟墙文件已上传到远程服务器");
                    } else {
                        QString error_output = upload_process->readAll();
                        ROS_WARN("上传虚拟墙文件到远程失败: %s", error_output.toStdString().c_str());
                    }
                }
                else
                {
                    ROS_WARN("上传虚拟墙文件到远程超时或启动失败");
                }
                
                // 安全清理进程对象
                upload_process->deleteLater();
            }
            else
            {
                ROS_WARN("SSH连接未建立，无法上传到远程");
            }
        }
        
        // 更新状态信息
        if (local_saved && remote_saved)
        {
            updateStatus("虚拟墙已清除并保存到本地和远程: " + base_name + ".json");
        }
        else if (local_saved)
        {
            updateStatus("虚拟墙已清除并保存到本地: " + base_name + ".json");
        }
        else
        {
            updateStatus("虚拟墙已清除，但保存文件失败", true);
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("自动保存虚拟墙文件时出错: %s", e.what());
    }
}

void MapEditPanel::saveRegionsFileAfterClear()
{
    try
    {
        // 获取当前地图文件路径
        std::string current_map_file = getCurrentMapFile();
        if (current_map_file.empty())
        {
            ROS_WARN("没有当前地图文件，无法自动保存区域文件");
            return;
        }
        
        // 构建区域文件路径
        QFileInfo map_info(QString::fromStdString(current_map_file));
        QString base_dir = map_info.absolutePath();
        QString base_name;
        
        // 如果是远程地图，使用存储的原始基础名称
        if (is_remote_map_ && !remote_map_base_name_.isEmpty()) {
            base_name = remote_map_base_name_;
        } else {
            base_name = map_info.baseName();
        }
        
        QString region_file = base_dir + "/" + base_name + "_region.json";
        
        // 保存空的区域文件（清除后的状态）
        std::vector<Region> empty_regions;
        bool local_saved = false;
        
        // 1. 保存到本地
        if (file_manager_->saveRegionsFile(region_file.toStdString(), empty_regions))
        {
            ROS_INFO("区域文件已保存到本地: %s", region_file.toStdString().c_str());
            local_saved = true;
        }
        else
        {
            ROS_WARN("保存本地区域文件失败: %s", file_manager_->getLastError().c_str());
        }
        
        // 2. 如果是远程地图，也保存到远程
        bool remote_saved = false;
        if (is_remote_map_ && !current_password_.isEmpty())
        {
            // 检查SSH连接状态
            if (connection_status_label_->text() == "连接成功!")
            {
                QString remote_host = ssh_host_edit_->text().trimmed();
                QString remote_user = ssh_username_edit_->text().trimmed();
                QString remote_dir = ssh_remote_dir_edit_->text().trimmed();
                
                // 构建远程文件路径
                QString remote_region_file = QString("%1@%2:%3/%4_region.json")
                                            .arg(remote_user)
                                            .arg(remote_host)
                                            .arg(remote_dir)
                                            .arg(base_name);
                
                ROS_INFO("正在上传区域文件到远程: %s", remote_region_file.toStdString().c_str());
                
                // 创建专用的SCP进程，避免与主进程冲突
                QProcess* upload_process = new QProcess(this);
                upload_process->setProcessChannelMode(QProcess::MergedChannels);
                
                // 构建SCP命令
                QString scp_command = QString("sshpass -p '%1' scp -o StrictHostKeyChecking=no -o ConnectTimeout=10 %2 %3")
                                     .arg(current_password_)
                                     .arg(region_file)
                                     .arg(remote_region_file);
                
                // 使用同步方式执行命令
                upload_process->start("bash", QStringList() << "-c" << scp_command);
                
                if (upload_process->waitForStarted(3000) && upload_process->waitForFinished(10000))
                {
                    if (upload_process->exitCode() == 0) {
                        remote_saved = true;
                        ROS_INFO("区域文件已上传到远程服务器");
                    } else {
                        QString error_output = upload_process->readAll();
                        ROS_WARN("上传区域文件到远程失败: %s", error_output.toStdString().c_str());
                    }
                }
                else
                {
                    ROS_WARN("上传区域文件到远程超时或启动失败");
                }
                
                // 安全清理进程对象
                upload_process->deleteLater();
            }
            else
            {
                ROS_WARN("SSH连接未建立，无法上传到远程");
            }
        }
        
        // 更新状态信息
        if (local_saved && remote_saved)
        {
            updateStatus("区域已清除并保存到本地和远程: " + base_name + "_region.json");
        }
        else if (local_saved)
        {
            updateStatus("区域已清除并保存到本地: " + base_name + "_region.json");
        }
        else
        {
            updateStatus("区域已清除，但保存文件失败", true);
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("自动保存区域文件时出错: %s", e.what());
    }
}

} // end namespace ros_map_edit

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros_map_edit::MapEditPanel, rviz::Panel) 