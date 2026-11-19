#ifndef BOT_RVIZ2_PANEL__GOAL_POSE_UPDATER_HPP_
#define BOT_RVIZ2_PANEL__GOAL_POSE_UPDATER_HPP_

#include <QObject>

namespace bot_rviz_panel
{

class GoalPoseUpdater : public QObject
{
  Q_OBJECT

public:
  GoalPoseUpdater() {}
  ~GoalPoseUpdater() {}

  void setGoal(double x, double y, double theta, QString frame)
  {
    emit updateGoal(x, y, theta, frame);
  }

signals:
  void updateGoal(double x, double y, double theta, QString frame);
};

} 

#endif 
