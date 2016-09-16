#include "rviz_ortho_view_controller/ortho_view_controller.h"

#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include <pluginlib/class_list_macros.h>

#include <QEvent>

#include <rviz/display_context.h>
#include <rviz/geometry.h>
#include <rviz/ogre_helpers/orthographic.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/viewport_mouse_event.h>

#include <ros/console.h>

namespace rviz_ortho_view_controller
{
static const double VIEW_DISTANCE = 500.0;
static const double DEFAULT_SCALE = 100.0;

static const char *STATUS = "<b>Left-Click:</b> Rotate PY.  <b>Middle-Click:</b> Move XY.  "
                            "<b>Right-Click/Mouse Wheel:</b> Zoom.  <b>Shift</b>: More options.";

static const char *STATUS_SHIFT = "<b>Left-Click:</b> Rotate R.  <b>Middle-Click:</b> Move XY.  "
                                  "<b>Right-Click:</b> Move Z.  <b>Mouse Wheel:</b> Zoom.";

OrthoViewController::OrthoViewController()
  : plane_property_(new rviz::EnumProperty("Plane", "none", "Optionally lock the view to a plane", this)),
    centre_property_(new rviz::VectorProperty("Centre", Ogre::Vector3::ZERO, "The focal point of the camera", this)),
    yaw_property_(new rviz::FloatProperty("Yaw", 0, "Angle around X axis to rotate", this)),
    pitch_property_(new rviz::FloatProperty("Pitch", 0, "Angle around Y axis to rotate", this)),
    roll_property_(new rviz::FloatProperty("Roll", 0, "Angle around Z axis to rotate", this)),
    scale_property_(new rviz::FloatProperty("Scale", DEFAULT_SCALE, "How much to scale up the scene", this))
{
  plane_property_->addOption("none", PLANE_NONE);
  plane_property_->addOption("XY", PLANE_XY);
  plane_property_->addOption("XZ", PLANE_XZ);
  plane_property_->addOption("YZ", PLANE_YZ);

  connect(plane_property_, SIGNAL(changed()), this, SLOT(onPlaneChanged()));
}

OrthoViewController::~OrthoViewController()
{
}

void OrthoViewController::onInitialize()
{
  FramePositionTrackingViewController::onInitialize();

  camera_->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  camera_->setFixedYawAxis(false);

  centre_shape_.reset(new rviz::Shape(rviz::Shape::Sphere, context_->getSceneManager(), target_scene_node_));
  centre_shape_->setScale(Ogre::Vector3(0.05f, 0.05f, 0.01f));
  centre_shape_->setColor(1.0f, 1.0f, 0.0f, 0.5f);
  centre_shape_->getRootNode()->setVisible(false);
}

void OrthoViewController::handleMouseEvent(rviz::ViewportMouseEvent &e)
{
  bool moved = false;

  int dx = 0;
  int dy = 0;

  if (e.shift())
    setStatus(STATUS_SHIFT);
  else
    setStatus(STATUS);

  if (e.type == QEvent::MouseButtonPress)
  {
    dragging_ = true;
    centre_shape_->getRootNode()->setVisible(true);
  }
  else if (e.type == QEvent::MouseButtonRelease)
  {
    dragging_ = false;
    centre_shape_->getRootNode()->setVisible(false);
  }
  else if (e.type == QEvent::MouseMove && dragging_)
  {
    moved = true;

    dx = e.x - e.last_x;
    dy = e.y - e.last_y;
  }

  bool rotate_z = e.shift() || getPlane() != PLANE_NONE;
  auto rotate_cursor = rotate_z ? Rotate2D : Rotate3D;

  if (e.left())
  {
    setCursor(rotate_cursor);

    if (rotate_z)
    {
      roll_property_->setFloat(rviz::mapAngleTo0_2Pi(roll_property_->getFloat() + 0.005 * dx));
    }
    else
    {
      yaw_property_->setFloat(rviz::mapAngleTo0_2Pi(yaw_property_->getFloat() - 0.005 * dy));
      pitch_property_->setFloat(rviz::mapAngleTo0_2Pi(pitch_property_->getFloat() - 0.005 * dx));
    }
  }
  else if (e.middle() || (e.left() && e.shift()))
  {
    setCursor(MoveXY);

    auto scale = scale_property_->getFloat();
    auto movement = getOrientation() * Ogre::Vector3(-dx / scale, dy / scale, 0);

    centre_property_->add(movement);
  }
  else if (e.right() && !e.shift())
  {
    setCursor(Zoom);
    scale_property_->multiply(1 - 0.01 * dy);
  }
  else if (e.right() && e.shift())
  {
    setCursor(MoveZ);

    auto scale = scale_property_->getFloat();
    auto movement = getOrientation() * Ogre::Vector3(0, 0, dy / scale);

    centre_property_->add(movement);
  }
  else
  {
    setCursor(e.shift() ? MoveXY : rotate_cursor);
  }

  if (e.wheel_delta)
  {
    moved = true;
    scale_property_->multiply(1 + 0.001 * e.wheel_delta);
  }

  if (moved)
  {
    context_->queueRender();
    emitConfigChanged();
  }
}

void OrthoViewController::lookAt(const Ogre::Vector3 &p)
{
  centre_property_->setVector(p - target_scene_node_->getPosition());
}

void OrthoViewController::reset()
{
  plane_property_->setString("none");
  centre_property_->setVector(Ogre::Vector3::ZERO);
  yaw_property_->setFloat(0);
  pitch_property_->setFloat(0);
  roll_property_->setFloat(0);
  scale_property_->setFloat(DEFAULT_SCALE);
}

void OrthoViewController::mimic(rviz::ViewController *source)
{
  FramePositionTrackingViewController::mimic(source);

  if (auto *source_ortho = qobject_cast<OrthoViewController *>(source))
  {
    plane_property_->setString(source_ortho->plane_property_->getString());
    centre_property_->setVector(source_ortho->centre_property_->getVector());

    yaw_property_->setFloat(source_ortho->yaw_property_->getFloat());
    pitch_property_->setFloat(source_ortho->pitch_property_->getFloat());
    roll_property_->setFloat(source_ortho->roll_property_->getFloat());

    scale_property_->setFloat(source_ortho->scale_property_->getFloat());
  }
  else
  {
    centre_property_->setVector(source->getCamera()->getPosition());
  }
}

void OrthoViewController::update(float dt, float ros_dt)
{
  FramePositionTrackingViewController::update(dt, ros_dt);

  // Build the projection matrix.
  auto scale = scale_property_->getFloat();
  auto width = camera_->getViewport()->getActualWidth() / scale / 2;
  auto height = camera_->getViewport()->getActualHeight() / scale / 2;

  auto near = camera_->getNearClipDistance();
  auto far = camera_->getFarClipDistance();

  Ogre::Matrix4 projection;
  rviz::buildScaledOrthoMatrix(projection, -width, width, -height, height, near, far);

  camera_->setCustomProjectionMatrix(true, projection);

  // Set the camera pose.
  auto centre = centre_property_->getVector();
  auto orientation = getOrientation();

  camera_->setOrientation(orientation);
  camera_->setPosition(centre + orientation * Ogre::Vector3::UNIT_Z * VIEW_DISTANCE);

  centre_shape_->setPosition(centre);
}

void OrthoViewController::onPlaneChanged()
{
  auto plane = getPlane();
  bool locked = plane != PLANE_NONE;

  yaw_property_->setReadOnly(locked);
  yaw_property_->setHidden(locked);

  pitch_property_->setReadOnly(locked);
  pitch_property_->setHidden(locked);

  if (locked)
  {
    yaw_property_->setFloat(0);
    pitch_property_->setFloat(0);

    if (plane == PLANE_XZ)
      yaw_property_->setFloat(M_PI / 2);
    else if (plane == PLANE_YZ)
      pitch_property_->setFloat(M_PI / 2);
  }
}

Ogre::Quaternion OrthoViewController::getOrientation() const
{
  return Ogre::Quaternion(Ogre::Radian(yaw_property_->getFloat()), Ogre::Vector3::UNIT_X) *
         Ogre::Quaternion(Ogre::Radian(pitch_property_->getFloat()), Ogre::Vector3::UNIT_Y) *
         Ogre::Quaternion(Ogre::Radian(roll_property_->getFloat()), Ogre::Vector3::UNIT_Z);
}

OrthoViewController::Plane OrthoViewController::getPlane() const
{
  return static_cast<Plane>(plane_property_->getOptionInt());
}
}

PLUGINLIB_EXPORT_CLASS(rviz_ortho_view_controller::OrthoViewController, rviz::ViewController);
