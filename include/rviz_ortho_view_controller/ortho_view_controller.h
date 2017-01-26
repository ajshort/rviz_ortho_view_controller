#ifndef RVIZ_ORTHO_VIEW_CONTROLLER_ORTHO_VIEW_CONTROLLER_H
#define RVIZ_ORTHO_VIEW_CONTROLLER_ORTHO_VIEW_CONTROLLER_H

#include <rviz/frame_position_tracking_view_controller.h>

#include <memory>

namespace rviz
{
class EnumProperty;
class FloatProperty;
class QuaternionProperty;
class Shape;
class VectorProperty;
}

namespace rviz_ortho_view_controller
{
/// An orthographic view controller which can be optionally locked to a plane.
class OrthoViewController : public rviz::FramePositionTrackingViewController
{
  Q_OBJECT

public:
  OrthoViewController();
  ~OrthoViewController();

  void onInitialize() override;

  void handleMouseEvent(rviz::ViewportMouseEvent &e) override;

  void lookAt(const Ogre::Vector3 &p) override;

  void reset() override;

  void mimic(rviz::ViewController *source) override;

  void update(float dt, float ros_dt) override;

private slots:
  void onPlaneChanged();

private:
  enum Plane
  {
    PLANE_NONE = 0,
    PLANE_XY,
    PLANE_XZ,
    PLANE_YZ
  };

  /// Gets the plane this view controller is currently locked to.
  Plane getPlane() const;

  rviz::EnumProperty *plane_property_;
  rviz::VectorProperty *centre_property_;
  rviz::QuaternionProperty *orientation_property_;
  rviz::FloatProperty *scale_property_;

  bool dragging_ = false;

  std::unique_ptr<rviz::Shape> centre_shape_;
};
}

#endif // RVIZ_ORTHO_VIEW_CONTROLLER_ORTHO_VIEW_CONTROLLER_H
