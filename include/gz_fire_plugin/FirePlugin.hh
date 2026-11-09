#ifndef GZ_FIRE_PLUGIN_FIRE_PLUGIN_HH_
#define GZ_FIRE_PLUGIN_FIRE_PLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{

class FirePluginPrivate;

/// Fire / flame / smoke / burning system for Gazebo Harmonic.
class FirePlugin :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
  public: FirePlugin();

  public: ~FirePlugin() override;

  public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) override;

  public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

  private: std::unique_ptr<FirePluginPrivate> dataPtr;
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif