#include <GL/gl.h>
#include <unistd.h>
#include <cstring>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <ignition/plugin/Register.hh>

// Fortress MinimalScene renders with OGRE in one GL context, then Qt samples
// that shared texture in another. WSLg/D3D12 can present the previous contents.
class Tron2FrameSync : public gz::gui::Plugin
{
public:
  void LoadConfig(const tinyxml2::XMLElement *) override
  {
    if (access("/dev/dxg", F_OK) == 0)
      gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  }

protected:
  bool eventFilter(QObject *object, QEvent *event) override
  {
    if (event->type() == gz::gui::events::Render::kType) {
      if (!identified) {
        const auto *renderer = reinterpret_cast<const char *>(glGetString(GL_RENDERER));
        if (renderer) {
          d3d12 = std::strstr(renderer, "D3D12") != nullptr;
          identified = true;
        }
      }
      // The Render event runs on the producer thread after camera->Update().
      if (d3d12) glFinish();
    }
    return QObject::eventFilter(object, event);
  }

private:
  bool identified = false;
  bool d3d12 = false;
};
IGNITION_ADD_PLUGIN(Tron2FrameSync, gz::gui::Plugin)
