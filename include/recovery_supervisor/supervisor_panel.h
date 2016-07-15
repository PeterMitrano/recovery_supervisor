#pragma once

#include <rviz/panel.h>

namespace supervisor_panel
{
/**
 * @Brief this rviz panel shows the status as normally printed
 * the the console by recovery_supervisor. It shows if demonstrations
 * are enabled or disabled. It shows how many recoveries are occuring
 * and what level.
 */
class SupervisorPanel : public rviz::Panel
{
  Q_OBJECT
public:
  SupervisorPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;
}
}
